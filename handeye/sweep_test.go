package handeye

import (
	"math"
	"math/rand"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func testBounds() AbsoluteBounds {
	return AbsoluteBounds{
		X: AxisBounds{Min: -100, Max: 100},
		Y: AxisBounds{Min: -100, Max: 100},
		Z: AxisBounds{Min: 200, Max: 400},
	}
}

func TestLookAtRotationPointsAtTarget(t *testing.T) {
	from := r3.Vector{X: 100, Y: 0, Z: 400}
	target := r3.Vector{X: 0, Y: 0, Z: 0}

	rm, err := lookAtRotation(from, target, 0, r3.Vector{Z: 1})
	test.That(t, err, test.ShouldBeNil)

	ov := rm.OrientationVectorRadians()
	zDir := r3.Vector{X: ov.OX, Y: ov.OY, Z: ov.OZ}
	expected := target.Sub(from).Normalize()
	test.That(t, zDir.Distance(expected), test.ShouldBeLessThan, 1e-9)
}

func TestLookAtRotationDegenerateErrors(t *testing.T) {
	pos := r3.Vector{X: 10, Y: 20, Z: 30}
	_, err := lookAtRotation(pos, pos, 0, r3.Vector{Z: 1})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "coincides with target")
}

func TestWorkspaceBoundsIsZero(t *testing.T) {
	test.That(t, WorkspaceBounds{}.IsZero(), test.ShouldBeTrue)
	test.That(t, WorkspaceBounds{Mode: WorkspaceBoundsModeAbsolute}.IsZero(), test.ShouldBeFalse)
	test.That(t, WorkspaceBounds{Absolute: &AbsoluteBounds{}}.IsZero(), test.ShouldBeFalse)
	test.That(t, WorkspaceBounds{Relative: &AbsoluteBounds{}}.IsZero(), test.ShouldBeFalse)
}

func TestWorkspaceBoundsResolveRelative(t *testing.T) {
	w := WorkspaceBounds{
		Mode: WorkspaceBoundsModeRelativeToBoard,
		Relative: &AbsoluteBounds{
			X: AxisBounds{Min: -300, Max: 300},
			Y: AxisBounds{Min: -300, Max: 300},
			Z: AxisBounds{Min: 450, Max: 800},
		},
	}
	got := w.Resolve(r3.Vector{X: 100, Y: 1000, Z: -200})
	test.That(t, got.X, test.ShouldResemble, AxisBounds{Min: -200, Max: 400})
	test.That(t, got.Y, test.ShouldResemble, AxisBounds{Min: 700, Max: 1300})
	test.That(t, got.Z, test.ShouldResemble, AxisBounds{Min: 250, Max: 600})
}

func TestWorkspaceBoundsResolveAbsolute(t *testing.T) {
	abs := AbsoluteBounds{
		X: AxisBounds{Min: 10, Max: 20},
		Y: AxisBounds{Min: 30, Max: 40},
		Z: AxisBounds{Min: 50, Max: 60},
	}
	w := WorkspaceBounds{Mode: WorkspaceBoundsModeAbsolute, Absolute: &abs}
	// boardCenter must be ignored in absolute mode.
	got := w.Resolve(r3.Vector{X: 999, Y: 999, Z: 999})
	test.That(t, got, test.ShouldResemble, abs)
}

func TestValidateWorkspaceBounds(t *testing.T) {
	okAbs := &AbsoluteBounds{
		X: AxisBounds{Min: -1, Max: 1},
		Y: AxisBounds{Min: -1, Max: 1},
		Z: AxisBounds{Min: -1, Max: 1},
	}
	tests := []struct {
		name    string
		w       WorkspaceBounds
		wantErr string
	}{
		{"absolute ok", WorkspaceBounds{Mode: WorkspaceBoundsModeAbsolute, Absolute: okAbs}, ""},
		{"relative ok", WorkspaceBounds{Mode: WorkspaceBoundsModeRelativeToBoard, Relative: okAbs}, ""},
		{"mode missing", WorkspaceBounds{Absolute: okAbs}, `workspace_bounds.mode must be`},
		{"mode unknown", WorkspaceBounds{Mode: "bogus"}, `unrecognized`},
		{"absolute mode missing payload", WorkspaceBounds{Mode: WorkspaceBoundsModeAbsolute}, `workspace_bounds.absolute is missing`},
		{"relative mode missing payload", WorkspaceBounds{Mode: WorkspaceBoundsModeRelativeToBoard}, `workspace_bounds.relative_to_board is missing`},
		{"absolute mode both payloads", WorkspaceBounds{Mode: WorkspaceBoundsModeAbsolute, Absolute: okAbs, Relative: okAbs}, `remove workspace_bounds.relative_to_board`},
		{"relative mode both payloads", WorkspaceBounds{Mode: WorkspaceBoundsModeRelativeToBoard, Absolute: okAbs, Relative: okAbs}, `remove workspace_bounds.absolute`},
		{"absolute inverted axis", WorkspaceBounds{Mode: WorkspaceBoundsModeAbsolute, Absolute: &AbsoluteBounds{X: AxisBounds{Min: 5, Max: 5}, Y: okAbs.Y, Z: okAbs.Z}}, `absolute.x.min (5)`},
		{"relative inverted axis", WorkspaceBounds{Mode: WorkspaceBoundsModeRelativeToBoard, Relative: &AbsoluteBounds{X: okAbs.X, Y: AxisBounds{Min: 1, Max: -1}, Z: okAbs.Z}}, `relative_to_board.y.min (1)`},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			err := validateWorkspaceBounds(tt.w)
			if tt.wantErr == "" {
				test.That(t, err, test.ShouldBeNil)
				return
			}
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, tt.wantErr)
		})
	}
}

func TestDeriveDefaultBounds(t *testing.T) {
	b := deriveDefaultBounds(r3.Vector{X: 100, Y: 500, Z: 0})
	test.That(t, b.X, test.ShouldResemble, AxisBounds{Min: -100, Max: 300})
	test.That(t, b.Y, test.ShouldResemble, AxisBounds{Min: 300, Max: 700})
	test.That(t, b.Z, test.ShouldResemble, AxisBounds{Min: 250, Max: 550})
}

func TestGeneratePoseInBounds(t *testing.T) {
	target := r3.Vector{X: 0, Y: 0, Z: 0}
	bounds := testBounds()
	rng := rand.New(rand.NewSource(42)) //nolint:gosec // deterministic seed for reproducible tests

	for i := 0; i < 20; i++ {
		p := generatePose(target, bounds, rng, r3.Vector{Z: 1})
		pt := p.Point()
		test.That(t, pt.X, test.ShouldBeGreaterThanOrEqualTo, bounds.X.Min)
		test.That(t, pt.X, test.ShouldBeLessThanOrEqualTo, bounds.X.Max)
		test.That(t, pt.Y, test.ShouldBeGreaterThanOrEqualTo, bounds.Y.Min)
		test.That(t, pt.Y, test.ShouldBeLessThanOrEqualTo, bounds.Y.Max)
		test.That(t, pt.Z, test.ShouldBeGreaterThanOrEqualTo, bounds.Z.Min)
		test.That(t, pt.Z, test.ShouldBeLessThanOrEqualTo, bounds.Z.Max)
	}
}

func TestGeneratePoseAimsAtTarget(t *testing.T) {
	target := r3.Vector{X: 0, Y: 0, Z: 0}
	rng := rand.New(rand.NewSource(42)) //nolint:gosec // deterministic seed for reproducible tests

	for i := 0; i < 10; i++ {
		p := generatePose(target, testBounds(), rng, r3.Vector{Z: 1})
		ov := p.Orientation().OrientationVectorRadians()
		zDir := r3.Vector{X: ov.OX, Y: ov.OY, Z: ov.OZ}
		expected := target.Sub(p.Point()).Normalize()
		test.That(t, zDir.Distance(expected), test.ShouldBeLessThan, 1e-6)
	}
}

func TestLookAtRotationArbitraryForwardAimsForwardAtTarget(t *testing.T) {
	from := r3.Vector{X: 100, Y: 0, Z: 0}
	target := r3.Vector{X: 0, Y: 0, Z: 0}
	// forward = local -Z: the "back" of the local frame should point at target.
	forward := r3.Vector{Z: -1}
	rm, err := lookAtRotation(from, target, 0, forward)
	test.That(t, err, test.ShouldBeNil)

	expectedWorld := target.Sub(from).Normalize()
	aa := rm.AxisAngles()
	// R @ forward in world should equal expectedWorld.
	actualWorld := applyRotation(rm, forward)
	test.That(t, actualWorld.Distance(expectedWorld), test.ShouldBeLessThan, 1e-9)
	_ = aa
}

func applyRotation(rm *spatialmath.RotationMatrix, v r3.Vector) r3.Vector {
	// RotationMatrix is a 3x3 row-major; use OrientationVectorRadians or just extract manually.
	// Easiest: build a pose and transform the vector.
	p := spatialmath.NewPose(r3.Vector{}, rm)
	// Transforming an input pose at (v) through p — but p is a rotation only, so result point = R*v.
	out := spatialmath.Compose(p, spatialmath.NewPoseFromPoint(v))
	return out.Point()
}

func TestGeneratePoseRollVaries(t *testing.T) {
	// Narrow bounds force near-identical positions so orientation differences
	// come from roll variation, not position variation.
	target := r3.Vector{X: 0, Y: 0, Z: 0}
	bounds := AbsoluteBounds{
		X: AxisBounds{Min: 50, Max: 51},
		Y: AxisBounds{Min: 50, Max: 51},
		Z: AxisBounds{Min: 300, Max: 301},
	}
	rng := rand.New(rand.NewSource(42)) //nolint:gosec // deterministic seed for reproducible tests

	poses := make([]r3.Vector, 10)
	for i := range poses {
		p := generatePose(target, bounds, rng, r3.Vector{Z: 1})
		ov := p.Orientation().OrientationVectorRadians()
		poses[i] = r3.Vector{X: ov.Theta}
	}

	someDifferent := false
	for _, p := range poses[1:] {
		if math.Abs(p.X-poses[0].X) > 0.1 {
			someDifferent = true
			break
		}
	}
	test.That(t, someDifferent, test.ShouldBeTrue)
}
