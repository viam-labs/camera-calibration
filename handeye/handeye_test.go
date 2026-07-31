package handeye

import (
	"context"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
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
