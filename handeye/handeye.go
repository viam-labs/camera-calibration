// Package handeye implements a Viam service that calibrates a camera
// mounted on a robotic arm (eye-in-hand). It consumes a pose_tracker,
// sweeps the arm through generated poses, and solves for the
// camera-in-gripper transform.
package handeye

import (
	"context"
	"errors"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/camera-calibration/internal/verb"
)

// Model is the resource model for the handeye calibration service.
var Model = resource.NewModel("viam", "camera-calibration", "handeye")

var errNotImplemented = errors.New("handeye calibration: not implemented")

func init() {
	resource.RegisterService(generic.API, Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newHandeye,
		},
	)
}

// Config is the attribute configuration for the handeye calibration service.
type Config struct {
	Arm         string `json:"arm"`
	PoseTracker string `json:"pose_tracker"`
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
	return []string{cfg.Arm, cfg.PoseTracker}, nil, nil
}

type handeye struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name    resource.Name
	logger  logging.Logger
	arm     arm.Arm
	tracker posetracker.PoseTracker
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
		arm:     a,
		tracker: tr,
	}, nil
}

func (h *handeye) Name() resource.Name {
	return h.name
}

func (h *handeye) DoCommand(
	_ context.Context,
	cmd map[string]interface{},
) (map[string]interface{}, error) {
	v, err := verb.Single(cmd)
	if err != nil {
		return nil, err
	}
	switch v {
	case "calibrate", "cancel", "result":
		return nil, errNotImplemented
	default:
		return nil, fmt.Errorf("unknown verb %q; expected calibrate, cancel, or result", v)
	}
}

// seed reads arm pose + board detection, returning a nominal board-in-base estimate.
func (h *handeye) seed(ctx context.Context) (map[string]interface{}, error) {
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

	return map[string]interface{}{
		"arm_joints_rad":          jointsToFloats(joints),
		"arm_end_position_mm":     poseToJSON(endPose),
		"board_pose_in_camera_mm": poseToJSON(boardInCamera),
		"board_pose_in_base_mm":   poseToJSON(boardInBase),
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
