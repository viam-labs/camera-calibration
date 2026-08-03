package handeye

import (
	"math"
	"math/rand"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"
)

func testBounds() WorkspaceBounds {
	return WorkspaceBounds{
		X: AxisBounds{Min: -100, Max: 100},
		Y: AxisBounds{Min: -100, Max: 100},
		Z: AxisBounds{Min: 200, Max: 400},
	}
}

func TestLookAtRotationPointsAtTarget(t *testing.T) {
	from := r3.Vector{X: 100, Y: 0, Z: 400}
	target := r3.Vector{X: 0, Y: 0, Z: 0}

	rm, err := lookAtRotation(from, target, 0)
	test.That(t, err, test.ShouldBeNil)

	ov := rm.OrientationVectorRadians()
	zDir := r3.Vector{X: ov.OX, Y: ov.OY, Z: ov.OZ}
	expected := target.Sub(from).Normalize()
	test.That(t, zDir.Distance(expected), test.ShouldBeLessThan, 1e-9)
}

func TestLookAtRotationDegenerateErrors(t *testing.T) {
	pos := r3.Vector{X: 10, Y: 20, Z: 30}
	_, err := lookAtRotation(pos, pos, 0)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "coincides with target")
}

func TestGeneratePoseInBounds(t *testing.T) {
	target := r3.Vector{X: 0, Y: 0, Z: 0}
	bounds := testBounds()
	rng := rand.New(rand.NewSource(42)) //nolint:gosec // deterministic seed for reproducible tests

	for i := 0; i < 20; i++ {
		p := generatePose(target, bounds, rng)
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
		p := generatePose(target, testBounds(), rng)
		ov := p.Orientation().OrientationVectorRadians()
		zDir := r3.Vector{X: ov.OX, Y: ov.OY, Z: ov.OZ}
		expected := target.Sub(p.Point()).Normalize()
		test.That(t, zDir.Distance(expected), test.ShouldBeLessThan, 1e-6)
	}
}

func TestGeneratePoseRollVaries(t *testing.T) {
	// Narrow bounds force near-identical positions so orientation differences
	// come from roll variation, not position variation.
	target := r3.Vector{X: 0, Y: 0, Z: 0}
	bounds := WorkspaceBounds{
		X: AxisBounds{Min: 50, Max: 51},
		Y: AxisBounds{Min: 50, Max: 51},
		Z: AxisBounds{Min: 300, Max: 301},
	}
	rng := rand.New(rand.NewSource(42)) //nolint:gosec // deterministic seed for reproducible tests

	poses := make([]r3.Vector, 10)
	for i := range poses {
		p := generatePose(target, bounds, rng)
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
