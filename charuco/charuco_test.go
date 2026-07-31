package charuco

import (
	"testing"

	"go.viam.com/test"
)

// validCfg is a minimal valid config used as a starting point for tests.
func validCfg() Config {
	return Config{
		Camera:         "cam",
		Dictionary:     "DICT_5X5_100",
		SquaresX:       7,
		SquaresY:       5,
		SquareLengthMM: 40.0,
		MarkerLengthMM: 30.0,
	}
}

func TestConfigValidate(t *testing.T) {
	tests := []struct {
		name    string
		mutate  func(*Config)
		wantErr string
	}{
		{
			name:    "valid",
			mutate:  func(_ *Config) {},
			wantErr: "",
		},
		{
			name:    "missing camera",
			mutate:  func(c *Config) { c.Camera = "" },
			wantErr: `Error validating, missing required field. Path: "test" Field: "camera"`,
		},
		{
			name:    "missing dictionary",
			mutate:  func(c *Config) { c.Dictionary = "" },
			wantErr: `Error validating, missing required field. Path: "test" Field: "dictionary"`,
		},
		{
			name:    "invalid dictionary",
			mutate:  func(c *Config) { c.Dictionary = "DICT_BOGUS" },
			wantErr: `invalid dictionary "DICT_BOGUS"; must be one of DICT_{4X4,5X5,6X6,7X7}_{50,100,250,1000} or DICT_ARUCO_ORIGINAL`,
		},
		{
			name:    "squares_x too small",
			mutate:  func(c *Config) { c.SquaresX = 1 },
			wantErr: "squares_x must be >= 2, got 1",
		},
		{
			name:    "squares_y too small",
			mutate:  func(c *Config) { c.SquaresY = 0 },
			wantErr: "squares_y must be >= 2, got 0",
		},
		{
			name:    "square_length_mm zero",
			mutate:  func(c *Config) { c.SquareLengthMM = 0 },
			wantErr: "square_length_mm must be > 0, got 0",
		},
		{
			name:    "marker_length_mm zero",
			mutate:  func(c *Config) { c.MarkerLengthMM = 0 },
			wantErr: "marker_length_mm must be > 0, got 0",
		},
		{
			name:    "marker not smaller than square",
			mutate:  func(c *Config) { c.MarkerLengthMM = 40.0 },
			wantErr: "marker_length_mm (40) must be less than square_length_mm (40)",
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			cfg := validCfg()
			tt.mutate(&cfg)
			_, _, err := cfg.Validate("test")
			if tt.wantErr == "" {
				test.That(t, err, test.ShouldBeNil)
				return
			}
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldEqual, tt.wantErr)
		})
	}
}
