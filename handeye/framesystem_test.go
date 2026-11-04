package handeye

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/arm/kinematics"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

func TestBuildFrameSystemUsesService(t *testing.T) {
	fsSvc := inject.NewFrameSystemService("test")
	fsSvc.FrameSystemConfigFunc = func(_ context.Context) (*framesystem.Config, error) {
		return &framesystem.Config{Parts: nil}, nil
	}
	fsSvc.CurrentInputsFunc = func(_ context.Context) (referenceframe.FrameSystemInputs, error) {
		return referenceframe.FrameSystemInputs{}, nil
	}
	h := &handeye{
		logger:    logging.NewTestLogger(t),
		cfg:       &Config{},
		arm:       inject.NewArm("arm"),
		fsService: fsSvc,
	}
	fs, err := h.buildFrameSystem(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs, test.ShouldNotBeNil)
}

func TestBuildFrameSystemPropagatesServiceError(t *testing.T) {
	fsSvc := inject.NewFrameSystemService("test")
	fsSvc.FrameSystemConfigFunc = func(_ context.Context) (*framesystem.Config, error) {
		return nil, errors.New("boom")
	}
	h := &handeye{
		logger:    logging.NewTestLogger(t),
		cfg:       &Config{},
		fsService: fsSvc,
	}
	_, err := h.buildFrameSystem(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "build frame system")
	test.That(t, err.Error(), test.ShouldContainSubstring, "boom")
}

func TestBuildFrameSystemAppliesInputRangeOverride(t *testing.T) {
	fsSvc := inject.NewFrameSystemService("test")
	fsSvc.FrameSystemConfigFunc = func(_ context.Context) (*framesystem.Config, error) {
		return &framesystem.Config{Parts: nil}, nil
	}
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg: &Config{
			InputRangeOverride: map[string]referenceframe.Limit{
				"0": {Min: -1, Max: 1},
			},
		},
		arm:       armWithModel(makeTestArmModel(t), "nonexistent-arm"),
		fsService: fsSvc,
	}
	_, err := h.buildFrameSystem(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "input_range_override")
}

const gantryTestGeomLabel = "rail"

// gantryCell is world → gantry-1 (prismatic +x, 0–3000 mm, at x=0) → arm (UR5e).
func gantryCell(t *testing.T) (*handeye, *[][]referenceframe.Input) {
	t.Helper()
	rail, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: 1500, Z: -100}), r3.Vector{X: 3200, Y: 200, Z: 50}, gantryTestGeomLabel)
	test.That(t, err, test.ShouldBeNil)
	slide, err := referenceframe.NewTranslationalFrameWithGeometry("slide", r3.Vector{X: 1}, referenceframe.Limit{Min: 0, Max: 3000}, rail)
	test.That(t, err, test.ShouldBeNil)
	gantryModel, err := referenceframe.NewSerialModel("gantry-1", []referenceframe.Frame{slide})
	test.That(t, err, test.ShouldBeNil)
	armModel, err := kinematics.ModelFromName(kinematics.UR5e, "arm")
	test.That(t, err, test.ShouldBeNil)

	fsSvc := inject.NewFrameSystemService("test")
	fsSvc.FrameSystemConfigFunc = func(_ context.Context) (*framesystem.Config, error) {
		return &framesystem.Config{Parts: []*referenceframe.FrameSystemPart{
			{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "gantry-1", nil), ModelFrame: gantryModel},
			{FrameConfig: referenceframe.NewLinkInFrame("gantry-1", spatialmath.NewZeroPose(), "arm", nil), ModelFrame: armModel},
		}}, nil
	}
	fsSvc.CurrentInputsFunc = func(_ context.Context) (referenceframe.FrameSystemInputs, error) {
		return referenceframe.FrameSystemInputs{
			"gantry-1": {0},
			"arm":      {0, -math.Pi / 2, math.Pi / 2, -math.Pi / 2, -math.Pi / 2, 0},
		}, nil
	}

	var executed [][]referenceframe.Input
	a := inject.NewArm("arm")
	a.MoveThroughJointPositionsFunc = func(_ context.Context, positions [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{}) error {
		executed = positions
		return nil
	}
	return &handeye{
		logger:    logging.NewTestLogger(t),
		cfg:       &Config{SaveSlowPlanThresholdMs: math.MaxInt32},
		arm:       a,
		fsService: fsSvc,
	}, &executed
}

func TestPlanAndExecuteCannotUseGantryToReachTarget(t *testing.T) {
	h, executed := gantryCell(t)
	fs, err := h.buildFrameSystem(context.Background())
	test.That(t, err, test.ShouldBeNil)

	outOfArmReach := spatialmath.NewPose(r3.Vector{X: 2000, Y: 300, Z: 300}, &spatialmath.OrientationVectorDegrees{OZ: -1})
	_, planErr, execErr := h.planAndExecute(context.Background(), fs, outOfArmReach, "test")
	test.That(t, execErr, test.ShouldBeNil)
	test.That(t, planErr, test.ShouldNotBeNil)
	test.That(t, planErr.Error(), test.ShouldContainSubstring, "too far")
	test.That(t, *executed, test.ShouldBeNil)
}

func TestPlanAndExecuteReachesTargetWithGantryFixed(t *testing.T) {
	h, executed := gantryCell(t)
	fs, err := h.buildFrameSystem(context.Background())
	test.That(t, err, test.ShouldBeNil)

	target := spatialmath.NewPose(r3.Vector{X: 400, Y: 300, Z: 300}, &spatialmath.OrientationVectorDegrees{OZ: -1})
	_, planErr, execErr := h.planAndExecute(context.Background(), fs, target, "test")
	test.That(t, planErr, test.ShouldBeNil)
	test.That(t, execErr, test.ShouldBeNil)
	test.That(t, *executed, test.ShouldNotBeEmpty)

	// Re-check the endpoint against the unfrozen system at the gantry's real position.
	raw, err := framesystem.NewFromService(context.Background(), h.fsService, nil)
	test.That(t, err, test.ShouldBeNil)
	final := (*executed)[len(*executed)-1]
	inputs := referenceframe.FrameSystemInputs{"gantry-1": {0}, "arm": final}.ToLinearInputs()
	got, err := raw.Transform(inputs, referenceframe.NewPoseInFrame("arm", spatialmath.NewZeroPose()), referenceframe.World)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.PoseAlmostCoincidentEps(got.(*referenceframe.PoseInFrame).Pose(), target, 1), test.ShouldBeTrue)
}

func TestFreezeNonArmFramesKeepsGantryGeometryInPlace(t *testing.T) {
	h, _ := gantryCell(t)
	inputs, err := h.fsService.CurrentInputs(context.Background())
	test.That(t, err, test.ShouldBeNil)
	raw, err := framesystem.NewFromService(context.Background(), h.fsService, nil)
	test.That(t, err, test.ShouldBeNil)
	before, err := referenceframe.FrameSystemGeometries(raw, inputs)
	test.That(t, err, test.ShouldBeNil)

	fs, err := h.buildFrameSystem(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.Frame("gantry-1").DoF(), test.ShouldBeEmpty)
	test.That(t, len(fs.Frame("arm").DoF()), test.ShouldEqual, 6)
	after, err := referenceframe.FrameSystemGeometries(fs, referenceframe.FrameSystemInputs{"arm": inputs["arm"]})
	test.That(t, err, test.ShouldBeNil)

	world := func(m map[string]*referenceframe.GeometriesInFrame) spatialmath.Pose {
		for _, gif := range m {
			for _, g := range gif.Geometries() {
				if g.Label() == gantryTestGeomLabel || g.Label() == "gantry-1:"+gantryTestGeomLabel {
					return g.Pose()
				}
			}
		}
		t.Fatalf("rail geometry not found in %v", m)
		return nil
	}
	test.That(t, spatialmath.PoseAlmostEqual(world(before), world(after)), test.ShouldBeTrue)
}

func TestPlanAndExecuteRejectsPlanThatMovesNonArmFrame(t *testing.T) {
	h, executed := gantryCell(t)
	unfrozen, err := framesystem.NewFromService(context.Background(), h.fsService, nil)
	test.That(t, err, test.ShouldBeNil)

	outOfArmReach := spatialmath.NewPose(r3.Vector{X: 2000, Y: 300, Z: 300}, &spatialmath.OrientationVectorDegrees{OZ: -1})
	_, planErr, execErr := h.planAndExecute(context.Background(), unfrozen, outOfArmReach, "test")
	test.That(t, planErr, test.ShouldBeNil)
	test.That(t, execErr, test.ShouldNotBeNil)
	test.That(t, execErr.Error(), test.ShouldContainSubstring, `plan moves frame "gantry-1"`)
	test.That(t, *executed, test.ShouldBeNil)
}
