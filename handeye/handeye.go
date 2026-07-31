// Package handeye implements a Viam service that calibrates a camera
// mounted on a robotic arm (eye-in-hand). It consumes a pose_tracker,
// sweeps the arm through generated poses, and solves for the
// camera-in-gripper transform.
package handeye

import (
	"context"
	"errors"
	"fmt"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
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

	name   resource.Name
	logger logging.Logger
}

func newHandeye(
	_ context.Context,
	_ resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	return &handeye{name: conf.ResourceName(), logger: logger}, nil
}

func (h *handeye) Name() resource.Name {
	return h.name
}

func (h *handeye) DoCommand(
	_ context.Context,
	cmd map[string]interface{},
) (map[string]interface{}, error) {
	for verb := range cmd {
		switch verb {
		case "calibrate", "cancel", "result":
			return nil, errNotImplemented
		default:
			return nil, fmt.Errorf("unknown verb %q; expected calibrate, cancel, or result", verb)
		}
	}
	return nil, errors.New("no verb provided in DoCommand")
}

func (h *handeye) Status(_ context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}
