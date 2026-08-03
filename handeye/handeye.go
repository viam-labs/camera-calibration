// Package handeye implements a Viam service that calibrates a camera
// mounted on a robotic arm (eye-in-hand). It consumes a pose_tracker,
// sweeps the arm through generated poses, and solves for the
// camera-in-gripper transform.
package handeye

import (
	"context"
	"errors"
	"fmt"
	"math/rand"
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
)

// Model is the resource model for the handeye calibration service.
var Model = resource.NewModel("viam", "camera-calibration", "handeye")

func init() {
	resource.RegisterService(generic.API, Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newHandeye,
		},
	)
}

// Config is the attribute configuration for the handeye calibration service.
type Config struct {
	Arm             string          `json:"arm"`
	PoseTracker     string          `json:"pose_tracker"`
	NumPoses        int             `json:"num_poses"`
	WorkspaceBounds WorkspaceBounds `json:"workspace_bounds"`
	SettleSeconds   float64         `json:"settle_seconds"`
}

// Validate checks that required fields are set and returns the implicit
// dependencies for the service.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if cfg.PoseTracker == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "pose_tracker")
	}
	if cfg.NumPoses == 0 {
		cfg.NumPoses = 20
	}
	if cfg.NumPoses < 3 {
		return nil, nil, fmt.Errorf("num_poses must be >= 3, got %d", cfg.NumPoses)
	}
	for _, axis := range []struct {
		name string
		b    AxisBounds
	}{
		{"x", cfg.WorkspaceBounds.X},
		{"y", cfg.WorkspaceBounds.Y},
		{"z", cfg.WorkspaceBounds.Z},
	} {
		if axis.b.Min >= axis.b.Max {
			return nil, nil, fmt.Errorf("workspace_bounds.%s.min (%g) must be < max (%g)", axis.name, axis.b.Min, axis.b.Max)
		}
	}
	if cfg.SettleSeconds == 0 {
		cfg.SettleSeconds = 2.0
	}
	if cfg.SettleSeconds < 0 {
		return nil, nil, fmt.Errorf("settle_seconds must be > 0, got %g", cfg.SettleSeconds)
	}
	return []string{cfg.Arm, cfg.PoseTracker}, nil, nil
}

type handeye struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name    resource.Name
	logger  logging.Logger
	cfg     *Config
	arm     arm.Arm
	tracker posetracker.PoseTracker

	mu         sync.Mutex
	cancelFn   context.CancelFunc     // non-nil while calibrate is running
	lastResult map[string]interface{} // cached result of last successful calibrate
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
	a, err := arm.FromProvider(deps, cfg.Arm)
	if err != nil {
		return nil, fmt.Errorf("handeye: get arm dep %q: %w", cfg.Arm, err)
	}
	tr, err := posetracker.FromProvider(deps, cfg.PoseTracker)
	if err != nil {
		return nil, fmt.Errorf("handeye: get pose_tracker dep %q: %w", cfg.PoseTracker, err)
	}
	return &handeye{
		name:    conf.ResourceName(),
		logger:  logger,
		cfg:     cfg,
		arm:     a,
		tracker: tr,
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
	case "result":
		return h.result()
	default:
		return nil, fmt.Errorf("unknown verb %q; expected calibrate, cancel, or result", v)
	}
}

// calibrate runs seed, pose generation, and sweep. Only one calibrate
// may run at a time; concurrent invocations error.
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

	seedRes, err := h.seed(calibCtx)
	if err != nil {
		return nil, fmt.Errorf("handeye: seed: %w", err)
	}
	seedCenter := seedRes.BoardInBase.Point()
	seedRvec := seedRes.BoardInBase.Orientation().AxisAngles().ToR3()
	h.logger.Infof("handeye: seed board_in_base center=(%.1f, %.1f, %.1f) rvec=(%.3f, %.3f, %.3f)",
		seedCenter.X, seedCenter.Y, seedCenter.Z, seedRvec.X, seedRvec.Y, seedRvec.Z)

	//nolint:gosec // sampling for pose generation, not crypto
	rng := rand.New(rand.NewSource(time.Now().UnixNano()))
	poses, err := generatePoses(seedCenter, h.cfg.NumPoses, h.cfg.WorkspaceBounds, rng)
	if err != nil {
		return nil, fmt.Errorf("handeye: generate poses: %w", err)
	}

	stations, err := h.sweepAndCapture(calibCtx, poses)
	if err != nil {
		return nil, fmt.Errorf("handeye: sweep: %w", err)
	}

	result := map[string]interface{}{
		"num_stations": len(stations),
		"stations":     stations,
	}
	h.mu.Lock()
	h.lastResult = result
	h.mu.Unlock()
	return result, nil
}

// cancel is idempotent: if a calibrate is running, its context is cancelled
// and arm.Stop is invoked (ctx cancellation may not reach the arm in time
// to halt an in-flight motion). If nothing is running, it's a no-op success
// so the caller doesn't have to check state before hitting stop.
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

func (h *handeye) result() (map[string]interface{}, error) {
	h.mu.Lock()
	defer h.mu.Unlock()
	if h.lastResult == nil {
		return nil, errors.New("handeye: no calibrate has completed yet")
	}
	return h.lastResult, nil
}

// sweepAndCapture runs the plan/execute/settle/capture loop over each
// target pose. Plan failures skip; execute failures halt (arm state unknown).
func (h *handeye) sweepAndCapture(ctx context.Context, poses []spatialmath.Pose) ([]map[string]interface{}, error) {
	armModel, err := h.arm.Kinematics(ctx)
	if err != nil {
		return nil, fmt.Errorf("handeye: get arm kinematics: %w", err)
	}
	fs := referenceframe.NewEmptyFrameSystem("handeye")
	if err := fs.AddFrame(armModel, fs.World()); err != nil {
		return nil, fmt.Errorf("handeye: build frame system: %w", err)
	}

	stations := make([]map[string]interface{}, 0, len(poses))
	settleDur := time.Duration(h.cfg.SettleSeconds * float64(time.Second))

	for i, target := range poses {
		if err := ctx.Err(); err != nil {
			return nil, err
		}

		planErr, execErr := h.planAndExecute(ctx, fs, target)
		if execErr != nil {
			// Motion failure (estop, pstop, collision, comms loss, ...) —
			// arm state is unknown, do not continue sweeping.
			return nil, fmt.Errorf("handeye: pose %d execute failed, halting sweep: %w", i, execErr)
		}
		if planErr != nil {
			h.logger.Infof("handeye: pose %d unreachable, skipping: %v", i, planErr)
			continue
		}

		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-time.After(settleDur):
		}

		detectResp, err := h.tracker.DoCommand(ctx, map[string]interface{}{"detect": map[string]interface{}{}})
		if err != nil {
			h.logger.Infof("handeye: pose %d detect failed, skipping: %v", i, err)
			continue
		}
		boardInCamera, err := parseBoardPose(detectResp)
		if err != nil {
			h.logger.Infof("handeye: pose %d board not detected, skipping", i)
			continue
		}

		actualEndPose, err := h.arm.EndPosition(ctx, nil)
		if err != nil {
			return nil, fmt.Errorf("handeye: read arm pose at station %d: %w", i, err)
		}

		stations = append(stations, map[string]interface{}{
			"T_be": poseToJSON(actualEndPose),
			"T_cw": poseToJSON(boardInCamera),
		})
	}
	return stations, nil
}

// planAndExecute returns (planErr, execErr) so callers can distinguish
// "unreachable pose, safe to skip" from "motion failed mid-execution,
// halt for safety".
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

// seed reads arm pose + board detection, returning a nominal board-in-base estimate.
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
	return map[string]interface{}{}, nil
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
