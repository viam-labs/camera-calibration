package charuco

import (
	"context"
	"encoding/json"
	"testing"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

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

func validIntrinsics() *transform.PinholeCameraIntrinsics {
	return &transform.PinholeCameraIntrinsics{
		Width: 1400, Height: 1000,
		Fx: 1000.0, Fy: 1000.0,
		Ppx: 700.0, Ppy: 500.0,
	}
}

func zeroDistortion() *transform.BrownConrady {
	return &transform.BrownConrady{}
}

func cameraWithIntrinsics() *inject.Camera {
	cam := &inject.Camera{}
	cam.PropertiesFunc = func(_ context.Context) (camera.Properties, error) {
		return camera.Properties{
			IntrinsicParams:  validIntrinsics(),
			DistortionParams: zeroDistortion(),
		}, nil
	}
	return cam
}

func TestConfigValidate(t *testing.T) {
	tests := []struct {
		name    string
		mutate  func(*Config)
		wantErr string
	}{
		{name: "valid", mutate: func(_ *Config) {}, wantErr: ""},
		{name: "missing camera", mutate: func(c *Config) { c.Camera = "" }, wantErr: `Error validating, missing required field. Path: "test" Field: "camera"`},
		{name: "missing dictionary", mutate: func(c *Config) { c.Dictionary = "" }, wantErr: `Error validating, missing required field. Path: "test" Field: "dictionary"`},
		{name: "invalid dictionary", mutate: func(c *Config) { c.Dictionary = "DICT_BOGUS" }, wantErr: `invalid dictionary "DICT_BOGUS"; must be one of DICT_{4X4,5X5,6X6,7X7}_{50,100,250,1000} or DICT_ARUCO_ORIGINAL`},
		{name: "squares_x too small", mutate: func(c *Config) { c.SquaresX = 1 }, wantErr: "squares_x must be >= 2, got 1"},
		{name: "squares_y too small", mutate: func(c *Config) { c.SquaresY = 0 }, wantErr: "squares_y must be >= 2, got 0"},
		{name: "square_length_mm zero", mutate: func(c *Config) { c.SquareLengthMM = 0 }, wantErr: "square_length_mm must be > 0, got 0"},
		{name: "marker_length_mm zero", mutate: func(c *Config) { c.MarkerLengthMM = 0 }, wantErr: "marker_length_mm must be > 0, got 0"},
		{name: "marker not smaller than square", mutate: func(c *Config) { c.MarkerLengthMM = 40.0 }, wantErr: "marker_length_mm (40) must be less than square_length_mm (40)"},
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

func TestNewCharucoResolvesCamera(t *testing.T) {
	cfg := validCfg()
	deps := resource.Dependencies{
		camera.Named(cfg.Camera): cameraWithIntrinsics(),
	}
	conf := resource.Config{
		Name:                "test-charuco",
		API:                 resource.APINamespaceRDK.WithComponentType("pose_tracker"),
		ConvertedAttributes: &cfg,
	}
	r, err := newCharuco(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, r, test.ShouldNotBeNil)
}

func TestIntrinsicsJSON(t *testing.T) {
	c := &charuco{
		cfg:    &Config{Camera: "cam"},
		camera: cameraWithIntrinsics(),
		logger: logging.NewTestLogger(t),
	}
	got, err := c.intrinsicsJSON(context.Background(), 1400, 1000)
	test.That(t, err, test.ShouldBeNil)

	var parsed map[string]interface{}
	test.That(t, json.Unmarshal([]byte(got), &parsed), test.ShouldBeNil)
	test.That(t, parsed["fx"], test.ShouldEqual, 1000.0)
	test.That(t, parsed["fy"], test.ShouldEqual, 1000.0)
	test.That(t, parsed["cx"], test.ShouldEqual, 700.0)
	test.That(t, parsed["cy"], test.ShouldEqual, 500.0)
	test.That(t, parsed["distortion"], test.ShouldNotBeNil)
}

func TestIntrinsicsJSONScalesToImageResolution(t *testing.T) {
	c := &charuco{
		cfg:    &Config{Camera: "cam"},
		camera: cameraWithIntrinsics(),
		logger: logging.NewTestLogger(t),
	}
	got, err := c.intrinsicsJSON(context.Background(), 700, 500)
	test.That(t, err, test.ShouldBeNil)

	var parsed map[string]interface{}
	test.That(t, json.Unmarshal([]byte(got), &parsed), test.ShouldBeNil)
	test.That(t, parsed["fx"], test.ShouldEqual, 500.0)
	test.That(t, parsed["fy"], test.ShouldEqual, 500.0)
	test.That(t, parsed["cx"], test.ShouldEqual, 350.0)
	test.That(t, parsed["cy"], test.ShouldEqual, 250.0)
}

func TestIntrinsicsJSONMissingResolutionErrors(t *testing.T) {
	cam := &inject.Camera{}
	cam.PropertiesFunc = func(_ context.Context) (camera.Properties, error) {
		params := validIntrinsics()
		params.Width = 0
		params.Height = 0
		return camera.Properties{IntrinsicParams: params, DistortionParams: zeroDistortion()}, nil
	}
	c := &charuco{
		cfg:    &Config{Camera: "cam"},
		camera: cam,
		logger: logging.NewTestLogger(t),
	}
	_, err := c.intrinsicsJSON(context.Background(), 1280, 720)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "intrinsic resolution unknown")
}

func TestIntrinsicsJSONMissingIntrinsics(t *testing.T) {
	cam := &inject.Camera{}
	cam.PropertiesFunc = func(_ context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}
	c := &charuco{
		cfg:    &Config{Camera: "cam"},
		camera: cam,
		logger: logging.NewTestLogger(t),
	}
	_, err := c.intrinsicsJSON(context.Background(), 1400, 1000)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "missing or zero intrinsics")
}

func TestIntrinsicsJSONMissingDistortion(t *testing.T) {
	cam := &inject.Camera{}
	cam.PropertiesFunc = func(_ context.Context) (camera.Properties, error) {
		return camera.Properties{IntrinsicParams: validIntrinsics()}, nil
	}
	c := &charuco{
		cfg:    &Config{Camera: "cam"},
		camera: cam,
		logger: logging.NewTestLogger(t),
	}
	_, err := c.intrinsicsJSON(context.Background(), 1400, 1000)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no distortion parameters")
}

func TestScaleIntrinsicsNoOp(t *testing.T) {
	fx, fy, cx, cy, err := scaleIntrinsics(1000, 1000, 700, 500, 1400, 1000, 1400, 1000)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fx, test.ShouldEqual, 1000.0)
	test.That(t, fy, test.ShouldEqual, 1000.0)
	test.That(t, cx, test.ShouldEqual, 700.0)
	test.That(t, cy, test.ShouldEqual, 500.0)
}

func TestScaleIntrinsicsRescales(t *testing.T) {
	fx, fy, cx, cy, err := scaleIntrinsics(1500, 1500, 960, 540, 1920, 1080, 1280, 720)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fx, test.ShouldEqual, 1000.0)
	test.That(t, fy, test.ShouldEqual, 1000.0)
	test.That(t, cx, test.ShouldEqual, 640.0)
	test.That(t, cy, test.ShouldEqual, 360.0)
}

func TestScaleIntrinsicsUnknownIntrinsicResolutionErrors(t *testing.T) {
	_, _, _, _, err := scaleIntrinsics(1000, 1000, 700, 500, 0, 0, 1280, 720)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "intrinsic resolution unknown")
}

func TestScaleIntrinsicsInvalidImageDimsErrors(t *testing.T) {
	_, _, _, _, err := scaleIntrinsics(1000, 1000, 700, 500, 1400, 1000, 0, 720)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "image dimensions invalid")
}

func TestDoCommandUnknownVerb(t *testing.T) {
	c := &charuco{
		name:   resource.Name{},
		logger: logging.NewTestLogger(t),
	}
	_, err := c.DoCommand(context.Background(), map[string]interface{}{"bogus": nil})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, `unknown verb "bogus"`)
}

func TestDoCommandNoVerb(t *testing.T) {
	c := &charuco{
		name:   resource.Name{},
		logger: logging.NewTestLogger(t),
	}
	_, err := c.DoCommand(context.Background(), map[string]interface{}{})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldEqual, "expected exactly one verb in DoCommand, got 0")
}

func TestDoCommandMultipleVerbs(t *testing.T) {
	c := &charuco{
		name:   resource.Name{},
		logger: logging.NewTestLogger(t),
	}
	_, err := c.DoCommand(context.Background(), map[string]interface{}{"detect": nil, "hint": true})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldEqual, "expected exactly one verb in DoCommand, got 2")
}
