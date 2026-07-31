package handeye

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

func TestConfigValidate(t *testing.T) {
	tests := []struct {
		name    string
		cfg     Config
		wantErr string
	}{
		{
			name:    "missing arm",
			cfg:     Config{PoseTracker: "tracker"},
			wantErr: `Error validating, missing required field. Path: "test" Field: "arm"`,
		},
		{
			name:    "missing pose_tracker",
			cfg:     Config{Arm: "arm"},
			wantErr: `Error validating, missing required field. Path: "test" Field: "pose_tracker"`,
		},
		{
			name:    "valid",
			cfg:     Config{Arm: "arm", PoseTracker: "tracker"},
			wantErr: "",
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			_, _, err := tt.cfg.Validate("test")
			if tt.wantErr == "" {
				test.That(t, err, test.ShouldBeNil)
				return
			}
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldEqual, tt.wantErr)
		})
	}
}

func TestDoCommand(t *testing.T) {
	tests := []struct {
		name    string
		cmd     map[string]interface{}
		wantErr string
	}{
		{
			name:    "calibrate returns not implemented",
			cmd:     map[string]interface{}{"calibrate": nil},
			wantErr: "handeye calibration: not implemented",
		},
		{
			name:    "cancel returns not implemented",
			cmd:     map[string]interface{}{"cancel": nil},
			wantErr: "handeye calibration: not implemented",
		},
		{
			name:    "result returns not implemented",
			cmd:     map[string]interface{}{"result": nil},
			wantErr: "handeye calibration: not implemented",
		},
		{
			name:    "unknown verb rejected",
			cmd:     map[string]interface{}{"bogus": nil},
			wantErr: `unknown verb "bogus"; expected calibrate, cancel, or result`,
		},
		{
			name:    "empty command rejected",
			cmd:     map[string]interface{}{},
			wantErr: "expected exactly one verb in DoCommand, got 0",
		},
		{
			name:    "multiple verbs rejected",
			cmd:     map[string]interface{}{"calibrate": nil, "cancel": nil},
			wantErr: "expected exactly one verb in DoCommand, got 2",
		},
	}

	h := &handeye{
		name:   resource.Name{},
		logger: logging.NewTestLogger(t),
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			resp, err := h.DoCommand(context.Background(), tt.cmd)
			test.That(t, resp, test.ShouldBeNil)
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldEqual, tt.wantErr)
		})
	}
}

func TestSeedReturnsExpectedShape(t *testing.T) {
	a := &inject.Arm{}
	a.JointPositionsFunc = func(_ context.Context, _ map[string]interface{}) ([]referenceframe.Input, error) {
		return []referenceframe.Input{0.1, 0.2, 0.3}, nil
	}
	a.EndPositionFunc = func(_ context.Context, _ map[string]interface{}) (spatialmath.Pose, error) {
		return spatialmath.NewPoseFromPoint(r3.Vector{X: 100, Y: 200, Z: 300}), nil
	}

	tr := &inject.PoseTracker{}
	tr.DoFunc = func(_ context.Context, _ map[string]interface{}) (map[string]interface{}, error) {
		return map[string]interface{}{
			"board_pose_mm": map[string]interface{}{
				"translation": []interface{}{10.0, 20.0, 400.0},
				"rvec":        []interface{}{0.1, 0.0, 0.0},
			},
		}, nil
	}

	h := &handeye{
		name:    resource.Name{},
		logger:  logging.NewTestLogger(t),
		arm:     a,
		tracker: tr,
	}

	resp, err := h.seed(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["arm_joints_rad"], test.ShouldResemble, []float64{0.1, 0.2, 0.3})
	test.That(t, resp["arm_end_position_mm"], test.ShouldNotBeNil)
	test.That(t, resp["board_pose_in_camera_mm"], test.ShouldNotBeNil)
	test.That(t, resp["board_pose_in_base_mm"], test.ShouldNotBeNil)
}

func TestSeedBoardNotDetectedErrors(t *testing.T) {
	a := &inject.Arm{}
	a.JointPositionsFunc = func(_ context.Context, _ map[string]interface{}) ([]referenceframe.Input, error) {
		return []referenceframe.Input{0}, nil
	}
	a.EndPositionFunc = func(_ context.Context, _ map[string]interface{}) (spatialmath.Pose, error) {
		return spatialmath.NewZeroPose(), nil
	}

	tr := &inject.PoseTracker{}
	tr.DoFunc = func(_ context.Context, _ map[string]interface{}) (map[string]interface{}, error) {
		return map[string]interface{}{
			"num_detected":  0,
			"corners":       []interface{}{},
			"board_pose_mm": nil,
		}, nil
	}

	h := &handeye{
		name:    resource.Name{},
		logger:  logging.NewTestLogger(t),
		arm:     a,
		tracker: tr,
	}

	_, err := h.seed(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "board not detected")
}
