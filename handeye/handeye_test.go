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

func validCfg() Config {
	return Config{
		Arm:         "arm",
		PoseTracker: "tracker",
		NumPoses:    20,
		WorkspaceBounds: WorkspaceBounds{
			X: AxisBounds{Min: -200, Max: 200},
			Y: AxisBounds{Min: -200, Max: 200},
			Z: AxisBounds{Min: 100, Max: 500},
		},
		SettleSeconds: 2.0,
	}
}

func TestConfigValidate(t *testing.T) {
	tests := []struct {
		name    string
		mutate  func(*Config)
		wantErr string
	}{
		{name: "valid", mutate: func(_ *Config) {}, wantErr: ""},
		{name: "missing arm", mutate: func(c *Config) { c.Arm = "" }, wantErr: `Error validating, missing required field. Path: "test" Field: "arm"`},
		{name: "missing pose_tracker", mutate: func(c *Config) { c.PoseTracker = "" }, wantErr: `Error validating, missing required field. Path: "test" Field: "pose_tracker"`},
		{name: "num_poses too small", mutate: func(c *Config) { c.NumPoses = 2 }, wantErr: "num_poses must be >= 3, got 2"},
		{name: "num_poses unset defaults to 20", mutate: func(c *Config) { c.NumPoses = 0 }, wantErr: ""},
		{name: "workspace_bounds x invalid", mutate: func(c *Config) { c.WorkspaceBounds.X = AxisBounds{Min: 100, Max: 100} }, wantErr: "workspace_bounds.x.min (100) must be < max (100)"},
		{name: "workspace_bounds y inverted", mutate: func(c *Config) { c.WorkspaceBounds.Y = AxisBounds{Min: 200, Max: 100} }, wantErr: "workspace_bounds.y.min (200) must be < max (100)"},
		{name: "workspace_bounds z zero", mutate: func(c *Config) { c.WorkspaceBounds.Z = AxisBounds{} }, wantErr: "workspace_bounds.z.min (0) must be < max (0)"},
		{name: "settle_seconds unset defaults to 2", mutate: func(c *Config) { c.SettleSeconds = 0 }, wantErr: ""},
		{name: "settle_seconds negative", mutate: func(c *Config) { c.SettleSeconds = -1 }, wantErr: "settle_seconds must be > 0, got -1"},
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

func TestDoCommandDispatch(t *testing.T) {
	tests := []struct {
		name    string
		cmd     map[string]interface{}
		wantErr string
	}{
		{name: "unknown verb rejected", cmd: map[string]interface{}{"bogus": nil}, wantErr: `unknown verb "bogus"; expected calibrate, cancel, or result`},
		{name: "empty command rejected", cmd: map[string]interface{}{}, wantErr: "expected exactly one verb in DoCommand, got 0"},
		{name: "multiple verbs rejected", cmd: map[string]interface{}{"calibrate": nil, "cancel": nil}, wantErr: "expected exactly one verb in DoCommand, got 2"},
	}
	h := &handeye{name: resource.Name{}, logger: logging.NewTestLogger(t)}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			_, err := h.DoCommand(context.Background(), tt.cmd)
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldEqual, tt.wantErr)
		})
	}
}

func mockArm() *inject.Arm {
	a := &inject.Arm{}
	a.JointPositionsFunc = func(_ context.Context, _ map[string]interface{}) ([]referenceframe.Input, error) {
		return []referenceframe.Input{0.1, 0.2, 0.3}, nil
	}
	a.EndPositionFunc = func(_ context.Context, _ map[string]interface{}) (spatialmath.Pose, error) {
		return spatialmath.NewPoseFromPoint(r3.Vector{X: 100, Y: 0, Z: 300}), nil
	}
	a.StopFunc = func(_ context.Context, _ map[string]interface{}) error { return nil }
	return a
}

func mockTracker() *inject.PoseTracker {
	tr := &inject.PoseTracker{}
	tr.DoFunc = func(_ context.Context, _ map[string]interface{}) (map[string]interface{}, error) {
		return map[string]interface{}{
			"board_pose_mm": map[string]interface{}{
				"translation": []interface{}{10.0, 20.0, 400.0},
				"rvec":        []interface{}{0.1, 0.0, 0.0},
			},
		}, nil
	}
	return tr
}

func TestSeedReturnsExpectedShape(t *testing.T) {
	h := &handeye{
		name:    resource.Name{},
		logger:  logging.NewTestLogger(t),
		arm:     mockArm(),
		tracker: mockTracker(),
	}
	resp, err := h.seed(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp.ArmJoints, test.ShouldResemble, []float64{0.1, 0.2, 0.3})
	test.That(t, resp.ArmEndPose, test.ShouldNotBeNil)
	test.That(t, resp.BoardInCamera, test.ShouldNotBeNil)
	test.That(t, resp.BoardInBase, test.ShouldNotBeNil)
}

func TestSeedBoardNotDetectedErrors(t *testing.T) {
	tr := &inject.PoseTracker{}
	tr.DoFunc = func(_ context.Context, _ map[string]interface{}) (map[string]interface{}, error) {
		return map[string]interface{}{"board_pose_mm": nil}, nil
	}
	h := &handeye{
		name:    resource.Name{},
		logger:  logging.NewTestLogger(t),
		arm:     mockArm(),
		tracker: tr,
	}
	_, err := h.seed(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "board not detected")
}

func TestResultBeforeCalibrateErrors(t *testing.T) {
	h := &handeye{name: resource.Name{}, logger: logging.NewTestLogger(t)}
	_, err := h.result()
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no calibrate has completed yet")
}

func TestCancelWithNoRunningCalibrateIsNoOp(t *testing.T) {
	stopCalled := false
	a := &inject.Arm{}
	a.StopFunc = func(_ context.Context, _ map[string]interface{}) error {
		stopCalled = true
		return nil
	}
	h := &handeye{name: resource.Name{}, logger: logging.NewTestLogger(t), arm: a}
	resp, err := h.cancel(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["cancelled"], test.ShouldEqual, true)
	test.That(t, resp["was_running"], test.ShouldEqual, false)
	test.That(t, stopCalled, test.ShouldBeFalse)
}

func TestCancelInvokesCancelFnAndStopsArm(t *testing.T) {
	stopCalled := false
	a := &inject.Arm{}
	a.StopFunc = func(_ context.Context, _ map[string]interface{}) error {
		stopCalled = true
		return nil
	}

	cancelCalled := false
	h := &handeye{
		name:     resource.Name{},
		logger:   logging.NewTestLogger(t),
		arm:      a,
		cancelFn: func() { cancelCalled = true },
	}

	resp, err := h.cancel(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cancelCalled, test.ShouldBeTrue)
	test.That(t, stopCalled, test.ShouldBeTrue)
	test.That(t, resp["was_running"], test.ShouldEqual, true)
}

func TestConcurrentCalibrateRejected(t *testing.T) {
	// Simulate a running calibrate by pre-setting cancelFn. calibrate must
	// error before touching arm/tracker, so no arm mocks are needed.
	_, existing := context.WithCancel(context.Background())
	h := &handeye{
		name:     resource.Name{},
		logger:   logging.NewTestLogger(t),
		cancelFn: existing,
	}
	_, err := h.calibrate(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "already running")
}
