// Package charuco implements a Viam pose_tracker component that detects
// ChArUco boards and returns each detected corner as a 6-DOF pose in the
// camera frame. Real detection is not yet implemented.
package charuco

import (
	"context"
	"errors"

	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
)

var Model = resource.NewModel("viam", "camera-calibration", "charuco")

var errNotImplemented = errors.New("charuco pose tracker: not implemented")

func init() {
	resource.RegisterComponent(posetracker.API, Model,
		resource.Registration[posetracker.PoseTracker, *Config]{
			Constructor: newCharuco,
		},
	)
}

type Config struct {
	Camera string `json:"camera"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Camera == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "camera")
	}
	return []string{cfg.Camera}, nil, nil
}

type charuco struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	logger logging.Logger
}

func newCharuco(
	_ context.Context,
	_ resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (posetracker.PoseTracker, error) {
	return &charuco{name: conf.ResourceName(), logger: logger}, nil
}

func (c *charuco) Name() resource.Name {
	return c.name
}

func (c *charuco) Poses(
	_ context.Context,
	_ []string,
	_ map[string]interface{},
) (referenceframe.FrameSystemPoses, error) {
	return nil, errNotImplemented
}

func (c *charuco) DoCommand(
	_ context.Context,
	_ map[string]interface{},
) (map[string]interface{}, error) {
	return nil, errNotImplemented
}
