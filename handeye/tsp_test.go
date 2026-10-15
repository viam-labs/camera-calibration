package handeye

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestPosePoolPopsNearestByCartesian(t *testing.T) {
	p := &posePool{}
	p.add(spatialmath.NewPoseFromPoint(r3.Vector{X: 1000}))
	p.add(spatialmath.NewPoseFromPoint(r3.Vector{X: 50}))
	p.add(spatialmath.NewPoseFromPoint(r3.Vector{X: 500}))

	chosen := p.popNearest(spatialmath.NewZeroPose())
	test.That(t, chosen.Point().X, test.ShouldEqual, 50.0)
	test.That(t, p.len(), test.ShouldEqual, 2)
}

func TestPosePoolOrientationCostContributes(t *testing.T) {
	// Same position, different orientations: pool should prefer the one closer in angle.
	nearOri, _ := spatialmath.NewOrientationVectorDegrees().Quaternion(), 0
	_ = nearOri
	closeOri := &spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: 1, Theta: 10}
	farOri := &spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: 1, Theta: 170}

	p := &posePool{}
	p.add(spatialmath.NewPose(r3.Vector{Z: 100}, farOri))
	p.add(spatialmath.NewPose(r3.Vector{Z: 100}, closeOri))

	ref := spatialmath.NewPose(r3.Vector{Z: 100}, &spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: 1, Theta: 0})
	chosen := p.popNearest(ref)
	chosenTheta := chosen.Orientation().OrientationVectorDegrees().Theta
	test.That(t, chosenTheta, test.ShouldAlmostEqual, 10.0, 1e-6)
}
