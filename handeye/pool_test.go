package handeye

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestPopBestFirstPickCollapsesToNearestTravel(t *testing.T) {
	p := &posePool{}
	p.add(spatialmath.NewPoseFromPoint(r3.Vector{X: 1000}))
	p.add(spatialmath.NewPoseFromPoint(r3.Vector{X: 50}))
	p.add(spatialmath.NewPoseFromPoint(r3.Vector{X: 500}))

	chosen := p.popBest(spatialmath.NewZeroPose(), nil)
	test.That(t, chosen.Point().X, test.ShouldEqual, 50.0)
}

func TestPopBestPrefersFarthestFromCaptured(t *testing.T) {
	// Two candidates: one near the already-captured pose, one far.
	// Travel costs are equal (both ~100 from current). Diversity should win.
	p := &posePool{}
	p.add(spatialmath.NewPoseFromPoint(r3.Vector{X: 100, Y: 10}))  // near captured
	p.add(spatialmath.NewPoseFromPoint(r3.Vector{X: 100, Y: -10})) // far from captured

	current := spatialmath.NewZeroPose()
	captured := []spatialmath.Pose{spatialmath.NewPoseFromPoint(r3.Vector{X: 100, Y: 500})}

	chosen := p.popBest(current, captured)
	test.That(t, chosen.Point().Y, test.ShouldEqual, -10.0)
}
