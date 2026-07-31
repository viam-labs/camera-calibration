package charuco

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
			name:    "missing camera",
			cfg:     Config{},
			wantErr: `error validating "test": expected "camera" attribute`,
		},
		{
			name:    "valid",
			cfg:     Config{Camera: "cam"},
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
