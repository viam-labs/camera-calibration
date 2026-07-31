package handeye

import (
	"fmt"
	"math"
	"math/rand"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// AxisBounds is a [min, max] range on a single axis, in mm.
type AxisBounds struct {
	Min float64 `json:"min"`
	Max float64 `json:"max"`
}

// WorkspaceBounds constrains the region (arm base frame, mm) that
// generated sweep poses will sample from.
type WorkspaceBounds struct {
	X AxisBounds `json:"x"`
	Y AxisBounds `json:"y"`
	Z AxisBounds `json:"z"`
}

// lookAtRotation builds a rotation whose local +Z axis points from `from`
// toward `target`, with an additional `rollRad` rotation about that axis.
// The returned matrix rotates local (camera) vectors into `from`/`target`'s frame.
func lookAtRotation(from, target r3.Vector, rollRad float64) (*spatialmath.RotationMatrix, error) {
	z := target.Sub(from)
	if z.Norm() < 1e-9 {
		return nil, fmt.Errorf("look-at position coincides with target")
	}
	z = z.Normalize()

	// Reference "world up"; fall back to +X if z is nearly parallel to +Z.
	worldUp := r3.Vector{Z: 1}
	if math.Abs(z.Dot(worldUp)) > 0.99 {
		worldUp = r3.Vector{X: 1}
	}

	xRef := worldUp.Cross(z).Normalize()
	yRef := z.Cross(xRef)

	c, s := math.Cos(rollRad), math.Sin(rollRad)
	x := xRef.Mul(c).Add(yRef.Mul(s))
	y := xRef.Mul(-s).Add(yRef.Mul(c))

	// Row-major 3x3 with columns [x, y, z].
	return spatialmath.NewRotationMatrix([]float64{
		x.X, y.X, z.X,
		x.Y, y.Y, z.Y,
		x.Z, y.Z, z.Z,
	})
}

// generatePoses samples numPoses arm-target poses aimed at `target` from
// positions uniformly sampled within `bounds`. Roll about the optical
// axis is uniform in [-π, π] — this breaks the AX=XB degenerate case
// where without roll variation, all rotation axes cluster in a plane
// perpendicular to the look-at direction.
func generatePoses(target r3.Vector, numPoses int, bounds WorkspaceBounds, rng *rand.Rand) ([]spatialmath.Pose, error) {
	if numPoses < 3 {
		return nil, fmt.Errorf("numPoses must be >= 3, got %d", numPoses)
	}
	poses := make([]spatialmath.Pose, 0, numPoses)
	for len(poses) < numPoses {
		pos := r3.Vector{
			X: bounds.X.Min + rng.Float64()*(bounds.X.Max-bounds.X.Min),
			Y: bounds.Y.Min + rng.Float64()*(bounds.Y.Max-bounds.Y.Min),
			Z: bounds.Z.Min + rng.Float64()*(bounds.Z.Max-bounds.Z.Min),
		}
		if pos.Sub(target).Norm() < 1e-6 {
			continue
		}
		roll := (rng.Float64()*2 - 1) * math.Pi
		rot, err := lookAtRotation(pos, target, roll)
		if err != nil {
			continue
		}
		poses = append(poses, spatialmath.NewPose(pos, rot))
	}
	return poses, nil
}
