package handeye

import (
	"context"
	"errors"
	"math"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
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
		{name: "num_poses too small", mutate: func(c *Config) { c.NumPoses = 2 }, wantErr: "num_poses must be >= 3 (or 0 for the default), got 2"},
		{name: "workspace_bounds x invalid", mutate: func(c *Config) { c.WorkspaceBounds.X = AxisBounds{Min: 100, Max: 100} }, wantErr: "workspace_bounds.x.min (100) must be < max (100); to auto-derive, omit the whole workspace_bounds block"},
		{name: "workspace_bounds y inverted", mutate: func(c *Config) { c.WorkspaceBounds.Y = AxisBounds{Min: 200, Max: 100} }, wantErr: "workspace_bounds.y.min (200) must be < max (100); to auto-derive, omit the whole workspace_bounds block"},
		{name: "workspace_bounds partial (only x set)", mutate: func(c *Config) { c.WorkspaceBounds.Y = AxisBounds{}; c.WorkspaceBounds.Z = AxisBounds{} }, wantErr: "workspace_bounds.y.min (0) must be < max (0); to auto-derive, omit the whole workspace_bounds block"},
		{name: "workspace_bounds fully omitted validates ok", mutate: func(c *Config) { c.WorkspaceBounds = WorkspaceBounds{} }, wantErr: ""},
		{name: "settle_seconds unset defaults to 2", mutate: func(c *Config) { c.SettleSeconds = 0 }, wantErr: ""},
		{name: "settle_seconds negative", mutate: func(c *Config) { c.SettleSeconds = -1 }, wantErr: "settle_seconds must be >= 0 (0 uses the default), got -1"},
		{name: "max_reprojection_error_px negative", mutate: func(c *Config) { c.MaxReprojectionErrorPx = -1 }, wantErr: "max_reprojection_error_px must be >= 0 (0 uses the default), got -1"},
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

func TestNumPosesZeroValidatesOK(t *testing.T) {
	cfg := validCfg()
	cfg.NumPoses = 0
	_, _, err := cfg.Validate("test")
	test.That(t, err, test.ShouldBeNil)
}

func TestSettleSecondsZeroValidatesOK(t *testing.T) {
	cfg := validCfg()
	cfg.SettleSeconds = 0
	_, _, err := cfg.Validate("test")
	test.That(t, err, test.ShouldBeNil)
}

func TestDoCommandDispatch(t *testing.T) {
	tests := []struct {
		name    string
		cmd     map[string]interface{}
		wantErr string
	}{
		{name: "unknown verb rejected", cmd: map[string]interface{}{"bogus": nil}, wantErr: `unknown verb "bogus"; expected calibrate or cancel`},
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

func makeTestArmModel(t *testing.T) *referenceframe.SimpleModel {
	t.Helper()
	j0, err := referenceframe.NewRotationalFrame("j0", spatialmath.R4AA{RZ: 1}, referenceframe.Limit{Min: -math.Pi, Max: math.Pi})
	test.That(t, err, test.ShouldBeNil)
	j1, err := referenceframe.NewRotationalFrame("j1", spatialmath.R4AA{RY: 1}, referenceframe.Limit{Min: -math.Pi, Max: math.Pi})
	test.That(t, err, test.ShouldBeNil)
	model, err := referenceframe.NewSerialModel("arm", []referenceframe.Frame{j0, j1})
	test.That(t, err, test.ShouldBeNil)
	return model
}

func armWithModel(model *referenceframe.SimpleModel, name string) *inject.Arm {
	a := inject.NewArm(name)
	a.KinematicsFunc = func(_ context.Context) (referenceframe.Model, error) { return model, nil }
	return a
}

func TestCheckStartingJointsSkipsWithNoOverride(t *testing.T) {
	h := &handeye{
		name:   resource.Name{},
		logger: logging.NewTestLogger(t),
		arm:    armWithModel(makeTestArmModel(t), "arm"),
		cfg:    &Config{},
	}
	err := h.checkStartingJointsInBounds(context.Background(), []referenceframe.Input{0.5, 0.5})
	test.That(t, err, test.ShouldBeNil)
}

func TestCheckStartingJointsAllowsInBounds(t *testing.T) {
	h := &handeye{
		name:   resource.Name{},
		logger: logging.NewTestLogger(t),
		arm:    armWithModel(makeTestArmModel(t), "arm"),
		cfg: &Config{
			InputRangeOverride: map[string]referenceframe.Limit{
				"0": {Min: -math.Pi / 2, Max: math.Pi / 2},
				"1": {Min: -math.Pi / 2, Max: math.Pi / 2},
			},
		},
	}
	err := h.checkStartingJointsInBounds(context.Background(), []referenceframe.Input{0.5, 0.5})
	test.That(t, err, test.ShouldBeNil)
}

func TestCheckStartingJointsErrorsWhenOutOfBounds(t *testing.T) {
	h := &handeye{
		name:   resource.Name{},
		logger: logging.NewTestLogger(t),
		arm:    armWithModel(makeTestArmModel(t), "arm"),
		cfg: &Config{
			InputRangeOverride: map[string]referenceframe.Limit{
				"0": {Min: -1.0472, Max: 1.5708},
			},
		},
	}
	err := h.checkStartingJointsInBounds(context.Background(), []referenceframe.Input{1.6992, 0.0})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm start position violates input_range_override")
	test.That(t, err.Error(), test.ShouldContainSubstring, "j0")
	test.That(t, err.Error(), test.ShouldContainSubstring, "1.6992")
}

func TestCheckArmHealthPassesWhenJointPositionsSucceeds(t *testing.T) {
	a := &inject.Arm{}
	a.JointPositionsFunc = func(_ context.Context, _ map[string]interface{}) ([]referenceframe.Input, error) {
		return []referenceframe.Input{0.1, 0.2, 0.3}, nil
	}
	h := &handeye{name: resource.Name{}, logger: logging.NewTestLogger(t), arm: a}
	test.That(t, h.checkArmHealth(context.Background(), 5), test.ShouldBeNil)
}

func TestCheckArmHealthErrorsWhenJointPositionsFails(t *testing.T) {
	a := &inject.Arm{}
	a.JointPositionsFunc = func(_ context.Context, _ map[string]interface{}) ([]referenceframe.Input, error) {
		return nil, errors.New("arm comms error")
	}
	h := &handeye{name: resource.Name{}, logger: logging.NewTestLogger(t), arm: a}
	err := h.checkArmHealth(context.Background(), 7)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "attempt 7")
	test.That(t, err.Error(), test.ShouldContainSubstring, "pstop, estop, or disconnected")
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm comms error")
}

func TestCheckStartingJointsSupportsJointNameKeys(t *testing.T) {
	h := &handeye{
		name:   resource.Name{},
		logger: logging.NewTestLogger(t),
		arm:    armWithModel(makeTestArmModel(t), "arm"),
		cfg: &Config{
			InputRangeOverride: map[string]referenceframe.Limit{
				"j1": {Min: -0.5, Max: 0.5},
			},
		},
	}
	err := h.checkStartingJointsInBounds(context.Background(), []referenceframe.Input{0.0, 1.0})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "j1")
}

func TestReturnToStartingPoseSkippedWhenCancelled(t *testing.T) {
	called := false
	a := &inject.Arm{}
	a.MoveThroughJointPositionsFunc = func(_ context.Context, _ [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{}) error {
		called = true
		return nil
	}
	h := &handeye{name: resource.Name{}, logger: logging.NewTestLogger(t), arm: a}
	cancelled, cancel := context.WithCancel(context.Background())
	cancel()
	h.returnToStartingPose(context.Background(), cancelled, []referenceframe.Input{0.1, 0.2, 0.3})
	test.That(t, called, test.ShouldBeFalse)
}

func TestReturnToStartingPoseMovesToJoints(t *testing.T) {
	var gotPositions [][]referenceframe.Input
	a := &inject.Arm{}
	a.MoveThroughJointPositionsFunc = func(_ context.Context, positions [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{}) error {
		gotPositions = positions
		return nil
	}
	h := &handeye{name: resource.Name{}, logger: logging.NewTestLogger(t), arm: a}
	starting := []referenceframe.Input{0.1, 0.2, 0.3}
	h.returnToStartingPose(context.Background(), context.Background(), starting)
	test.That(t, len(gotPositions), test.ShouldEqual, 1)
	test.That(t, gotPositions[0], test.ShouldResemble, starting)
}

func TestReturnToStartingPoseSwallowsMoveError(t *testing.T) {
	a := &inject.Arm{}
	a.MoveThroughJointPositionsFunc = func(_ context.Context, _ [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{}) error {
		return errors.New("arm broke")
	}
	h := &handeye{name: resource.Name{}, logger: logging.NewTestLogger(t), arm: a}
	h.returnToStartingPose(context.Background(), context.Background(), []referenceframe.Input{0.1})
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

func mustStatus(t *testing.T, h *handeye) map[string]interface{} {
	t.Helper()
	resp, err := h.Status(context.Background())
	test.That(t, err, test.ShouldBeNil)
	return resp
}

func TestStatusInitialReady(t *testing.T) {
	h := &handeye{
		name:     resource.Name{},
		logger:   logging.NewTestLogger(t),
		cfg:      &Config{NumPoses: 20},
		progress: progressState{state: "ready"},
	}
	resp := mustStatus(t, h)
	test.That(t, resp["state"], test.ShouldEqual, "ready")
	test.That(t, resp["positions_captured"], test.ShouldEqual, 0)
	test.That(t, resp["positions_required"], test.ShouldEqual, 20)
	test.That(t, resp["positions_attempted"], test.ShouldEqual, 0)
	test.That(t, resp, test.ShouldNotContainKey, "elapsed_time")
	test.That(t, resp, test.ShouldNotContainKey, "last_error")
	test.That(t, resp, test.ShouldNotContainKey, "translation")
}

func TestStatusDefaultsTotalPositionsWhenConfigZero(t *testing.T) {
	h := &handeye{
		name:     resource.Name{},
		logger:   logging.NewTestLogger(t),
		cfg:      &Config{NumPoses: 0},
		progress: progressState{state: "ready"},
	}
	resp := mustStatus(t, h)
	test.That(t, resp["positions_required"], test.ShouldEqual, 20)
}

func TestStatusIncludesFailureDetails(t *testing.T) {
	failedAt := time.Now()
	h := &handeye{
		name:   resource.Name{},
		logger: logging.NewTestLogger(t),
		cfg:    &Config{NumPoses: 20},
		progress: progressState{
			state:              "failed",
			positionsCaptured:  3,
			positionsAttempted: 8,
			startedAt:          failedAt.Add(-63 * time.Second),
			completedAt:        failedAt,
			lastError:          "handeye: something broke",
		},
	}
	resp := mustStatus(t, h)
	test.That(t, resp["state"], test.ShouldEqual, "failed")
	test.That(t, resp["positions_captured"], test.ShouldEqual, 3)
	test.That(t, resp["positions_attempted"], test.ShouldEqual, 8)
	test.That(t, resp["last_error"], test.ShouldEqual, "handeye: something broke")
	test.That(t, resp["elapsed_time"], test.ShouldEqual, "1m 3s")
}

func TestStatusIncludesResultWhenComplete(t *testing.T) {
	completedAt := time.Now()
	h := &handeye{
		name:   resource.Name{},
		logger: logging.NewTestLogger(t),
		cfg:    &Config{NumPoses: 20},
		progress: progressState{
			state:              "complete",
			positionsCaptured:  20,
			positionsAttempted: 22,
			startedAt:          completedAt.Add(-3 * time.Minute),
			completedAt:        completedAt,
		},
		lastResult: map[string]interface{}{
			"translation":             map[string]interface{}{"x": 1.0, "y": 2.0, "z": 3.0},
			"translation_residual_mm": 0.5,
		},
	}
	resp := mustStatus(t, h)
	test.That(t, resp["state"], test.ShouldEqual, "complete")
	test.That(t, resp["elapsed_time"], test.ShouldEqual, "3m 0s")
	test.That(t, resp["translation"], test.ShouldResemble, map[string]interface{}{"x": 1.0, "y": 2.0, "z": 3.0})
	test.That(t, resp["translation_residual_mm"], test.ShouldEqual, 0.5)
}

func TestFormatElapsed(t *testing.T) {
	test.That(t, formatElapsed(45*time.Second), test.ShouldEqual, "45s")
	test.That(t, formatElapsed(63*time.Second), test.ShouldEqual, "1m 3s")
	test.That(t, formatElapsed(3*time.Minute), test.ShouldEqual, "3m 0s")
	test.That(t, formatElapsed(3*time.Minute+15*time.Second), test.ShouldEqual, "3m 15s")
	test.That(t, formatElapsed(time.Hour+5*time.Minute+12*time.Second), test.ShouldEqual, "1h 5m 12s")
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

func makeTestArmFS(t *testing.T) *referenceframe.FrameSystem {
	t.Helper()
	j0, err := referenceframe.NewRotationalFrame("j0", spatialmath.R4AA{RZ: 1}, referenceframe.Limit{Min: -math.Pi, Max: math.Pi})
	test.That(t, err, test.ShouldBeNil)
	j1, err := referenceframe.NewRotationalFrame("j1", spatialmath.R4AA{RY: 1}, referenceframe.Limit{Min: -math.Pi, Max: math.Pi})
	test.That(t, err, test.ShouldBeNil)
	model, err := referenceframe.NewSerialModel("arm", []referenceframe.Frame{j0, j1})
	test.That(t, err, test.ShouldBeNil)
	fs := referenceframe.NewEmptyFrameSystem("test")
	test.That(t, fs.AddFrame(model, fs.World()), test.ShouldBeNil)
	return fs
}

func TestApplyJointLimitsTightensByJointName(t *testing.T) {
	fs := makeTestArmFS(t)
	overrides := map[string]referenceframe.Limit{"j0": {Min: -0.5, Max: 0.5}}
	err := applyJointLimits(logging.NewTestLogger(t), fs, "arm", overrides)
	test.That(t, err, test.ShouldBeNil)
	got := fs.Frame("arm").DoF()
	test.That(t, got[0], test.ShouldResemble, referenceframe.Limit{Min: -0.5, Max: 0.5})
	test.That(t, got[1], test.ShouldResemble, referenceframe.Limit{Min: -math.Pi, Max: math.Pi})
}

func TestApplyJointLimitsTightensByIndex(t *testing.T) {
	fs := makeTestArmFS(t)
	overrides := map[string]referenceframe.Limit{"1": {Min: -0.1, Max: 0.1}}
	err := applyJointLimits(logging.NewTestLogger(t), fs, "arm", overrides)
	test.That(t, err, test.ShouldBeNil)
	got := fs.Frame("arm").DoF()
	test.That(t, got[1], test.ShouldResemble, referenceframe.Limit{Min: -0.1, Max: 0.1})
}

func TestApplyJointLimitsClampsLoosening(t *testing.T) {
	fs := makeTestArmFS(t)
	overrides := map[string]referenceframe.Limit{"j0": {Min: -10, Max: 10}}
	err := applyJointLimits(logging.NewTestLogger(t), fs, "arm", overrides)
	test.That(t, err, test.ShouldBeNil)
	got := fs.Frame("arm").DoF()
	test.That(t, got[0], test.ShouldResemble, referenceframe.Limit{Min: -math.Pi, Max: math.Pi})
}

func TestApplyJointLimitsUnknownFrameErrors(t *testing.T) {
	fs := makeTestArmFS(t)
	overrides := map[string]referenceframe.Limit{"j0": {Min: -0.5, Max: 0.5}}
	err := applyJointLimits(logging.NewTestLogger(t), fs, "bogus", overrides)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "not found")
}

func TestApplyJointLimitsUnknownJointErrors(t *testing.T) {
	fs := makeTestArmFS(t)
	overrides := map[string]referenceframe.Limit{"bogus": {Min: -0.5, Max: 0.5}}
	err := applyJointLimits(logging.NewTestLogger(t), fs, "arm", overrides)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "can't find joint")
}

func TestSolveResponseToResult(t *testing.T) {
	resp := &solveResponse{}
	resp.CameraInGripperMM.Translation = []float64{10.0, 20.0, 30.0}
	resp.CameraInGripperMM.Rvec = []float64{0.0, 0.0, 0.0}
	resp.Residuals.TranslationMM = 0.5
	resp.Residuals.RotationDeg = 0.1
	resp.Residuals.PoseDiversityDeg = 90.0
	resp.Residuals.MeanStationReprojectionPx = 0.6
	resp.Residuals.MaxStationReprojectionPx = 0.9

	result := resp.toResult()

	trans, ok := result["translation"].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, trans["x"], test.ShouldEqual, 10.0)
	test.That(t, trans["y"], test.ShouldEqual, 20.0)
	test.That(t, trans["z"], test.ShouldEqual, 30.0)

	orient, ok := result["orientation"].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, orient["type"], test.ShouldEqual, "ov_degrees")
	ovValue, ok := orient["value"].(map[string]interface{})
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, ovValue, test.ShouldContainKey, "x")
	test.That(t, ovValue, test.ShouldContainKey, "y")
	test.That(t, ovValue, test.ShouldContainKey, "z")
	test.That(t, ovValue, test.ShouldContainKey, "th")

	test.That(t, result["translation_residual_mm"], test.ShouldEqual, 0.5)
	test.That(t, result["rotation_residual_deg"], test.ShouldEqual, 0.1)
	test.That(t, result["pose_diversity_deg"], test.ShouldEqual, 90.0)
	test.That(t, result["mean_station_reprojection_px"], test.ShouldEqual, 0.6)
	test.That(t, result["max_station_reprojection_px"], test.ShouldEqual, 0.9)
}
