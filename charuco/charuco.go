// Package charuco implements a Viam pose_tracker component that detects
// ChArUco boards and returns each detected corner as a 6-DOF pose in the
// camera frame. Real detection is not yet implemented.
package charuco

import (
	"context"
	"errors"
	"fmt"

	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
)

// Model is the resource model for the charuco pose tracker.
var Model = resource.NewModel("viam", "camera-calibration", "charuco")

var errNotImplemented = errors.New("charuco pose tracker: not implemented")

func init() {
	resource.RegisterComponent(posetracker.API, Model,
		resource.Registration[posetracker.PoseTracker, *Config]{
			Constructor: newCharuco,
		},
	)
}

// Config is the attribute configuration for the charuco pose tracker.
type Config struct {
	Camera         string  `json:"camera"`
	Dictionary     string  `json:"dictionary"`
	SquaresX       int     `json:"squares_x"`
	SquaresY       int     `json:"squares_y"`
	SquareLengthMM float64 `json:"square_length_mm"`
	MarkerLengthMM float64 `json:"marker_length_mm"`
}

// validDictionaries is the set of ChArUco dictionaries we accept. Names
// match OpenCV's cv2.aruco.DICT_* constants.
var validDictionaries = map[string]struct{}{
	"DICT_4X4_50":         {},
	"DICT_4X4_100":        {},
	"DICT_4X4_250":        {},
	"DICT_4X4_1000":       {},
	"DICT_5X5_50":         {},
	"DICT_5X5_100":        {},
	"DICT_5X5_250":        {},
	"DICT_5X5_1000":       {},
	"DICT_6X6_50":         {},
	"DICT_6X6_100":        {},
	"DICT_6X6_250":        {},
	"DICT_6X6_1000":       {},
	"DICT_7X7_50":         {},
	"DICT_7X7_100":        {},
	"DICT_7X7_250":        {},
	"DICT_7X7_1000":       {},
	"DICT_ARUCO_ORIGINAL": {},
}

// Validate checks that required fields are set and returns the implicit
// dependencies for the component.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Camera == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "camera")
	}
	if cfg.Dictionary == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "dictionary")
	}
	if _, ok := validDictionaries[cfg.Dictionary]; !ok {
		return nil, nil, fmt.Errorf(`invalid dictionary %q; must be one of DICT_{4X4,5X5,6X6,7X7}_{50,100,250,1000} or DICT_ARUCO_ORIGINAL`, cfg.Dictionary)
	}
	if cfg.SquaresX < 2 {
		return nil, nil, fmt.Errorf("squares_x must be >= 2, got %d", cfg.SquaresX)
	}
	if cfg.SquaresY < 2 {
		return nil, nil, fmt.Errorf("squares_y must be >= 2, got %d", cfg.SquaresY)
	}
	if cfg.SquareLengthMM <= 0 {
		return nil, nil, fmt.Errorf("square_length_mm must be > 0, got %g", cfg.SquareLengthMM)
	}
	if cfg.MarkerLengthMM <= 0 {
		return nil, nil, fmt.Errorf("marker_length_mm must be > 0, got %g", cfg.MarkerLengthMM)
	}
	if cfg.MarkerLengthMM >= cfg.SquareLengthMM {
		return nil, nil, fmt.Errorf("marker_length_mm (%g) must be less than square_length_mm (%g)", cfg.MarkerLengthMM, cfg.SquareLengthMM)
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

func (c *charuco) Status(_ context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}
