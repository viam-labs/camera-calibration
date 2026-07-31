package handeye

import (
	"testing"

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
