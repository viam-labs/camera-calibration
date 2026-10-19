package handeye

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestBoardDistanceAndTiltDegHeadOn(t *testing.T) {
	ov := &spatialmath.OrientationVector{OX: 0, OY: 0, OZ: -1, Theta: 0}
	pose := spatialmath.NewPose(r3.Vector{X: 0, Y: 0, Z: 1000}, ov)

	dist, tilt := boardDistanceAndTiltDeg(pose)
	test.That(t, dist, test.ShouldAlmostEqual, 1000.0, 1e-6)
	test.That(t, tilt, test.ShouldAlmostEqual, 0.0, 1e-6)
}

func TestBoardDistanceAndTiltDeg45(t *testing.T) {
	ov := &spatialmath.OrientationVector{OX: math.Sqrt(0.5), OY: 0, OZ: -math.Sqrt(0.5), Theta: 0}
	pose := spatialmath.NewPose(r3.Vector{X: 300, Y: 400, Z: 1200}, ov)

	dist, tilt := boardDistanceAndTiltDeg(pose)
	test.That(t, dist, test.ShouldAlmostEqual, math.Sqrt(300*300+400*400+1200*1200), 1e-6)
	test.That(t, tilt, test.ShouldAlmostEqual, 45.0, 1e-4)
}

func TestDetectCountsHandlesMissingKeys(t *testing.T) {
	nd, mc := detectCounts(map[string]interface{}{})
	test.That(t, nd, test.ShouldEqual, 0)
	test.That(t, mc, test.ShouldEqual, 0)

	nd, mc = detectCounts(map[string]interface{}{"num_detected": float64(80), "marker_count": float64(60)})
	test.That(t, nd, test.ShouldEqual, 80)
	test.That(t, mc, test.ShouldEqual, 60)
}
