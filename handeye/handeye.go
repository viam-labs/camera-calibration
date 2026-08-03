// Package handeye implements a Viam service that calibrates a camera
// mounted on a robotic arm (eye-in-hand). It consumes a pose_tracker,
// sweeps the arm through generated poses, and solves for the
// camera-in-gripper transform.
package handeye

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"strconv"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/camera-calibration/internal/verb"
	"github.com/viam-labs/camera-calibration/pyrunner"
)

// Model is the handeye calibration service.
var Model = resource.NewModel("viam", "camera-calibration", "handeye")

func init() {
	resource.RegisterService(generic.API, Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newHandeye,
		},
	)
}

// Config is the handeye service configuration.
type Config struct {
	Arm                string                                     `json:"arm"`
	PoseTracker        string                                     `json:"pose_tracker"`
	NumPoses           int                                        `json:"num_poses"`
	WorkspaceBounds    WorkspaceBounds                            `json:"workspace_bounds"`
	SettleSeconds      float64                                    `json:"settle_seconds"`
	InputRangeOverride map[string]map[string]referenceframe.Limit `json:"input_range_override,omitempty"`
}

// Validate returns implicit dependencies and any config errors.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if cfg.PoseTracker == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "pose_tracker")
	}
	if cfg.NumPoses != 0 && cfg.NumPoses < 3 {
		return nil, nil, fmt.Errorf("num_poses must be >= 3 (or 0 for the default), got %d", cfg.NumPoses)
	}
	if !cfg.WorkspaceBounds.IsZero() {
		for _, axis := range []struct {
			name string
			b    AxisBounds
		}{
			{"x", cfg.WorkspaceBounds.X},
			{"y", cfg.WorkspaceBounds.Y},
			{"z", cfg.WorkspaceBounds.Z},
		} {
			if axis.b.Min >= axis.b.Max {
				return nil, nil, fmt.Errorf("workspace_bounds.%s.min (%g) must be < max (%g); to auto-derive, omit the whole workspace_bounds block", axis.name, axis.b.Min, axis.b.Max)
			}
		}
	}
	if cfg.SettleSeconds < 0 {
		return nil, nil, fmt.Errorf("settle_seconds must be >= 0 (0 uses the default), got %g", cfg.SettleSeconds)
	}
	return []string{cfg.Arm, cfg.PoseTracker}, nil, nil
}

type handeye struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name      resource.Name
	logger    logging.Logger
	cfg       *Config
	arm       arm.Arm
	tracker   posetracker.PoseTracker
	pythonBin string
	solvePath string

	mu         sync.Mutex
	cancelFn   context.CancelFunc
	lastResult map[string]interface{}
	progress   progressState
}

type progressState struct {
	state             string
	positionsCaptured int
	attempts          int
	startedAt         time.Time
	completedAt       time.Time
	lastError         string
}

func newHandeye(
	_ context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	if cfg.NumPoses == 0 {
		cfg.NumPoses = 20
	}
	if cfg.SettleSeconds == 0 {
		cfg.SettleSeconds = 2.0
	}
	a, err := arm.FromProvider(deps, cfg.Arm)
	if err != nil {
		return nil, fmt.Errorf("handeye: get arm dep %q: %w", cfg.Arm, err)
	}
	tr, err := posetracker.FromProvider(deps, cfg.PoseTracker)
	if err != nil {
		return nil, fmt.Errorf("handeye: get pose_tracker dep %q: %w", cfg.PoseTracker, err)
	}
	exePath, err := os.Executable()
	if err != nil {
		return nil, fmt.Errorf("handeye: resolve executable path: %w", err)
	}
	moduleRoot := filepath.Dir(filepath.Dir(exePath))
	return &handeye{
		name:      conf.ResourceName(),
		logger:    logger,
		progress:  progressState{state: "ready"},
		cfg:       cfg,
		arm:       a,
		tracker:   tr,
		pythonBin: filepath.Join(moduleRoot, ".venv", "bin", "python"),
		solvePath: filepath.Join(moduleRoot, "python", "solve.py"),
	}, nil
}

func (h *handeye) Name() resource.Name {
	return h.name
}

func (h *handeye) DoCommand(
	ctx context.Context,
	cmd map[string]interface{},
) (map[string]interface{}, error) {
	v, err := verb.Single(cmd)
	if err != nil {
		return nil, err
	}
	switch v {
	case "calibrate":
		return h.calibrate(ctx)
	case "cancel":
		return h.cancel(ctx)
	default:
		return nil, fmt.Errorf("unknown verb %q; expected calibrate or cancel", v)
	}
}

func (h *handeye) calibrate(ctx context.Context) (map[string]interface{}, error) {
	calibCtx, cancel := context.WithCancel(ctx)
	h.mu.Lock()
	if h.cancelFn != nil {
		h.mu.Unlock()
		cancel()
		return nil, errors.New("handeye: calibrate already running")
	}
	h.cancelFn = cancel
	h.mu.Unlock()

	defer func() {
		h.mu.Lock()
		h.cancelFn = nil
		h.mu.Unlock()
	}()

	h.mu.Lock()
	h.progress = progressState{state: "capturing", startedAt: time.Now()}
	h.lastResult = nil
	h.mu.Unlock()

	fail := func(err error) error {
		h.mu.Lock()
		h.progress.state = "failed"
		h.progress.completedAt = time.Now()
		h.progress.lastError = err.Error()
		h.mu.Unlock()
		return err
	}

	seedRes, err := h.seed(calibCtx)
	if err != nil {
		return nil, fail(fmt.Errorf("handeye: seed: %w", err))
	}
	seedCenter := seedRes.BoardInBase.Point()
	seedRvec := seedRes.BoardInBase.Orientation().AxisAngles().ToR3()
	h.logger.Infof("handeye: seed board_in_base center=(%.1f, %.1f, %.1f) rvec=(%.3f, %.3f, %.3f)",
		seedCenter.X, seedCenter.Y, seedCenter.Z, seedRvec.X, seedRvec.Y, seedRvec.Z)

	if h.cfg.NumPoses < 3 {
		return nil, fail(fmt.Errorf("handeye: num_poses must be >= 3, got %d", h.cfg.NumPoses))
	}

	bounds := h.cfg.WorkspaceBounds
	if bounds.IsZero() {
		bounds = deriveDefaultBounds(seedCenter)
		h.logger.Infof("handeye: auto-derived workspace_bounds x=[%.1f, %.1f] y=[%.1f, %.1f] z=[%.1f, %.1f]",
			bounds.X.Min, bounds.X.Max, bounds.Y.Min, bounds.Y.Max, bounds.Z.Min, bounds.Z.Max)
	}

	//nolint:gosec // sampling for pose generation, not crypto
	rng := rand.New(rand.NewSource(time.Now().UnixNano()))
	nextPose := func() spatialmath.Pose {
		return generatePose(seedCenter, bounds, rng)
	}

	stations, err := h.sweepAndCapture(calibCtx, h.cfg.NumPoses, nextPose)
	if err != nil {
		return nil, fail(fmt.Errorf("handeye: sweep: %w", err))
	}

	h.mu.Lock()
	h.progress.state = "solving"
	h.mu.Unlock()

	result, err := h.runSolver(calibCtx, stations)
	if err != nil {
		return nil, fail(err)
	}

	h.mu.Lock()
	h.lastResult = result
	h.progress.state = "complete"
	h.progress.completedAt = time.Now()
	h.mu.Unlock()
	return result, nil
}

type solveResponse struct {
	CameraInGripperMM struct {
		Translation []float64 `json:"translation"`
		Rvec        []float64 `json:"rvec"`
	} `json:"camera_in_gripper_mm"`
	Residuals struct {
		TranslationMM float64 `json:"translation_mm"`
		RotationDeg   float64 `json:"rotation_deg"`
	} `json:"residuals"`
}

func (s *solveResponse) toResult() map[string]interface{} {
	t := s.CameraInGripperMM.Translation
	rvec := r3.Vector{X: s.CameraInGripperMM.Rvec[0], Y: s.CameraInGripperMM.Rvec[1], Z: s.CameraInGripperMM.Rvec[2]}
	ovd := spatialmath.R3ToR4(rvec).OrientationVectorDegrees()
	return map[string]interface{}{
		"translation": map[string]float64{"x": t[0], "y": t[1], "z": t[2]},
		"orientation": map[string]interface{}{
			"type": "ov_degrees",
			"value": map[string]float64{
				"x":  ovd.OX,
				"y":  ovd.OY,
				"z":  ovd.OZ,
				"th": ovd.Theta,
			},
		},
		"translation_residual_mm": s.Residuals.TranslationMM,
		"rotation_residual_deg":   s.Residuals.RotationDeg,
	}
}

func (h *handeye) runSolver(ctx context.Context, stations []map[string]interface{}) (map[string]interface{}, error) {
	payload, err := json.Marshal(map[string]interface{}{
		"stations": stations,
		"method":   "tsai",
	})
	if err != nil {
		return nil, fmt.Errorf("handeye: marshal solve input: %w", err)
	}

	tmpFile, err := os.CreateTemp("", "handeye-solve-*.json")
	if err != nil {
		return nil, fmt.Errorf("handeye: create solve input temp file: %w", err)
	}
	defer func() { _ = os.Remove(tmpFile.Name()) }()
	if _, werr := tmpFile.Write(payload); werr != nil {
		_ = tmpFile.Close()
		return nil, fmt.Errorf("handeye: write solve input: %w", werr)
	}
	if cerr := tmpFile.Close(); cerr != nil {
		return nil, fmt.Errorf("handeye: close solve input: %w", cerr)
	}

	stdout, err := pyrunner.Run(ctx, h.logger, h.pythonBin, h.solvePath, tmpFile.Name())
	if err != nil {
		return nil, fmt.Errorf("handeye: solve: %w", err)
	}

	var resp solveResponse
	if err := json.Unmarshal(stdout, &resp); err != nil {
		return nil, fmt.Errorf("handeye: parse solve output: %w", err)
	}
	if len(resp.CameraInGripperMM.Translation) != 3 || len(resp.CameraInGripperMM.Rvec) != 3 {
		return nil, fmt.Errorf("handeye: solve output missing translation or rvec")
	}
	return resp.toResult(), nil
}

func (h *handeye) cancel(ctx context.Context) (map[string]interface{}, error) {
	h.mu.Lock()
	cancel := h.cancelFn
	h.mu.Unlock()

	if cancel == nil {
		return map[string]interface{}{"cancelled": true, "was_running": false}, nil
	}
	cancel()
	if err := h.arm.Stop(ctx, nil); err != nil {
		h.logger.Warnf("handeye: arm.Stop failed during cancel: %v", err)
	}
	return map[string]interface{}{"cancelled": true, "was_running": true}, nil
}

func (h *handeye) sweepAndCapture(ctx context.Context, targetCount int, nextPose func() spatialmath.Pose) ([]map[string]interface{}, error) {
	armModel, err := h.arm.Kinematics(ctx)
	if err != nil {
		return nil, fmt.Errorf("handeye: get arm kinematics: %w", err)
	}
	fs := referenceframe.NewEmptyFrameSystem("handeye")
	if err := fs.AddFrame(armModel, fs.World()); err != nil {
		return nil, fmt.Errorf("handeye: build frame system: %w", err)
	}
	if len(h.cfg.InputRangeOverride) > 0 {
		if err := applyJointLimits(h.logger, fs, h.cfg.InputRangeOverride); err != nil {
			return nil, fmt.Errorf("handeye: apply input_range_override: %w", err)
		}
	}

	stations := make([]map[string]interface{}, 0, targetCount)
	settleDur := time.Duration(h.cfg.SettleSeconds * float64(time.Second))
	attempt := 0

	for len(stations) < targetCount {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		attempt++
		h.mu.Lock()
		h.progress.attempts = attempt
		h.mu.Unlock()
		target := nextPose()

		planErr, execErr := h.planAndExecute(ctx, fs, target)
		if execErr != nil {
			return nil, fmt.Errorf("handeye: attempt %d execute failed, halting sweep: %w", attempt, execErr)
		}
		if planErr != nil {
			h.logger.Infof("handeye: attempt %d unreachable, skipping: %v", attempt, planErr)
			continue
		}

		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-time.After(settleDur):
		}

		detectResp, err := h.tracker.DoCommand(ctx, map[string]interface{}{"detect": map[string]interface{}{}})
		if err != nil {
			h.logger.Infof("handeye: attempt %d detect failed, skipping: %v", attempt, err)
			continue
		}
		boardInCamera, err := parseBoardPose(detectResp)
		if err != nil {
			h.logger.Infof("handeye: attempt %d board not detected, skipping", attempt)
			continue
		}

		actualEndPose, err := h.arm.EndPosition(ctx, nil)
		if err != nil {
			return nil, fmt.Errorf("handeye: read arm pose at attempt %d: %w", attempt, err)
		}

		stations = append(stations, map[string]interface{}{
			"T_be": poseToJSON(actualEndPose),
			"T_cw": poseToJSON(boardInCamera),
		})
		h.mu.Lock()
		h.progress.positionsCaptured = len(stations)
		h.mu.Unlock()
	}
	return stations, nil
}

func (h *handeye) planAndExecute(ctx context.Context, fs *referenceframe.FrameSystem, target spatialmath.Pose) (planErr, execErr error) {
	armName := h.arm.Name().Name
	currentInputs, err := h.arm.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("get current joints: %w", err), nil
	}

	startState := armplanning.NewPlanState(nil, referenceframe.FrameSystemInputs{
		armName: currentInputs,
	})
	goalState := armplanning.NewPlanState(referenceframe.FrameSystemPoses{
		armName: referenceframe.NewPoseInFrame(referenceframe.World, target),
	}, nil)

	plan, _, err := armplanning.PlanMotion(ctx, h.logger, &armplanning.PlanRequest{
		FrameSystem: fs,
		StartState:  startState,
		Goals:       []*armplanning.PlanState{goalState},
	})
	if err != nil {
		return fmt.Errorf("plan motion: %w", err), nil
	}

	trajectory := make([][]referenceframe.Input, 0, len(plan.Trajectory()))
	for _, fsInputs := range plan.Trajectory() {
		armInputs, err := fsInputs.GetFrameInputs(fs.Frame(armName))
		if err != nil {
			return fmt.Errorf("extract arm inputs from trajectory: %w", err), nil
		}
		trajectory = append(trajectory, armInputs)
	}

	if err := h.arm.MoveThroughJointPositions(ctx, trajectory, nil, nil); err != nil {
		return nil, fmt.Errorf("execute trajectory: %w", err)
	}
	return nil, nil
}

type seedResult struct {
	ArmJoints     []float64
	ArmEndPose    spatialmath.Pose
	BoardInCamera spatialmath.Pose
	BoardInBase   spatialmath.Pose
}

func (h *handeye) seed(ctx context.Context) (*seedResult, error) {
	joints, err := h.arm.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("handeye: read arm joints: %w", err)
	}
	endPose, err := h.arm.EndPosition(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("handeye: read arm end position: %w", err)
	}

	detectResp, err := h.tracker.DoCommand(ctx, map[string]interface{}{"detect": map[string]interface{}{}})
	if err != nil {
		return nil, fmt.Errorf("handeye: pose_tracker detect: %w", err)
	}
	boardInCamera, err := parseBoardPose(detectResp)
	if err != nil {
		return nil, err
	}

	// Nominal camera-at-flange (X = identity); ~50-100mm off in practice, fine for sweep planning.
	boardInBase := spatialmath.Compose(endPose, boardInCamera)

	return &seedResult{
		ArmJoints:     jointsToFloats(joints),
		ArmEndPose:    endPose,
		BoardInCamera: boardInCamera,
		BoardInBase:   boardInBase,
	}, nil
}

func (h *handeye) Status(_ context.Context) (map[string]interface{}, error) {
	h.mu.Lock()
	p := h.progress
	result := h.lastResult
	h.mu.Unlock()

	total := h.cfg.NumPoses
	if total == 0 {
		total = 20
	}

	resp := map[string]interface{}{
		"state":              p.state,
		"positions_captured": p.positionsCaptured,
		"total_positions":    total,
		"attempts":           p.attempts,
	}
	if !p.startedAt.IsZero() {
		var elapsed time.Duration
		if p.completedAt.IsZero() {
			elapsed = time.Since(p.startedAt)
		} else {
			elapsed = p.completedAt.Sub(p.startedAt)
		}
		resp["elapsed_time"] = formatElapsed(elapsed)
	}
	if p.lastError != "" {
		resp["last_error"] = p.lastError
	}
	if p.state == "complete" && result != nil {
		for k, v := range result {
			resp[k] = v
		}
	}
	return resp, nil
}

func formatElapsed(d time.Duration) string {
	d = d.Round(time.Second)
	hours := int(d / time.Hour)
	d -= time.Duration(hours) * time.Hour
	mins := int(d / time.Minute)
	d -= time.Duration(mins) * time.Minute
	secs := int(d / time.Second)
	switch {
	case hours > 0:
		return fmt.Sprintf("%dh %dm %ds", hours, mins, secs)
	case mins > 0:
		return fmt.Sprintf("%dm %ds", mins, secs)
	default:
		return fmt.Sprintf("%ds", secs)
	}
}

func parseBoardPose(detectResp map[string]interface{}) (spatialmath.Pose, error) {
	raw, ok := detectResp["board_pose_mm"]
	if !ok || raw == nil {
		return nil, errors.New("handeye: board not detected (pose_tracker returned nil board_pose_mm)")
	}
	m, ok := raw.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("handeye: unexpected board_pose_mm type: %T", raw)
	}
	t, err := parseFloat3(m, "translation")
	if err != nil {
		return nil, err
	}
	rvec, err := parseFloat3(m, "rvec")
	if err != nil {
		return nil, err
	}
	return spatialmath.NewPose(
		r3.Vector{X: t[0], Y: t[1], Z: t[2]},
		spatialmath.R3ToR4(r3.Vector{X: rvec[0], Y: rvec[1], Z: rvec[2]}),
	), nil
}

func parseFloat3(m map[string]interface{}, key string) ([3]float64, error) {
	var out [3]float64
	raw, ok := m[key]
	if !ok {
		return out, fmt.Errorf("handeye: missing %q in board pose", key)
	}
	slice, ok := raw.([]interface{})
	if !ok || len(slice) != 3 {
		return out, fmt.Errorf("handeye: %q is not a 3-element array (got %T)", key, raw)
	}
	for i, v := range slice {
		f, ok := v.(float64)
		if !ok {
			return out, fmt.Errorf("handeye: %q[%d] is not a float (got %T)", key, i, v)
		}
		out[i] = f
	}
	return out, nil
}

func jointsToFloats(joints []referenceframe.Input) []float64 {
	out := make([]float64, len(joints))
	for i, j := range joints {
		out[i] = float64(j)
	}
	return out
}

func poseToJSON(p spatialmath.Pose) map[string]interface{} {
	pt := p.Point()
	rvec := p.Orientation().AxisAngles().ToR3()
	return map[string]interface{}{
		"translation": []float64{pt.X, pt.Y, pt.Z},
		"rvec":        []float64{rvec.X, rvec.Y, rvec.Z},
	}
}

func applyJointLimits(logger logging.Logger, fs *referenceframe.FrameSystem, inputRangeOverride map[string]map[string]referenceframe.Limit) error {
	for fName, mods := range inputRangeOverride {
		f := fs.Frame(fName)
		if f == nil {
			return fmt.Errorf("frame (%s) in input_range_override doesn't exist", fName)
		}
		sm, ok := f.(*referenceframe.SimpleModel)
		if !ok {
			return fmt.Errorf("can only override joints for SimpleModel for now, not %T", f)
		}

		resolved := make(map[string]referenceframe.Limit, len(mods))
		moveableNames := sm.MoveableFrameNames()
		existingLimits := sm.DoF()
		for key, limit := range mods {
			matched := false
			for i, name := range moveableNames {
				if key == name || key == strconv.Itoa(i) {
					existing := existingLimits[i]
					tightened := referenceframe.Limit{
						Min: math.Max(limit.Min, existing.Min),
						Max: math.Min(limit.Max, existing.Max),
					}
					if tightened.Min != limit.Min || tightened.Max != limit.Max {
						logger.Warnf(
							"input_range_override for frame %q joint %q would loosen limits: requested [%.6f, %.6f], model declares [%.6f, %.6f]; tightening to [%.6f, %.6f]",
							fName, name,
							limit.Min, limit.Max,
							existing.Min, existing.Max,
							tightened.Min, tightened.Max,
						)
					}
					resolved[name] = tightened
					matched = true
					break
				}
			}
			if !matched {
				return fmt.Errorf("can't find mod (%s)", key)
			}
		}

		newModel, err := referenceframe.NewModelWithLimitOverrides(sm, resolved)
		if err != nil {
			return err
		}
		if err := fs.ReplaceFrame(newModel); err != nil {
			return err
		}
	}
	return nil
}
