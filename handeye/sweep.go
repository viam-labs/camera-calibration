package handeye

import (
	"fmt"
	"math"
	"math/rand"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// AxisBounds is a [min, max] range in mm.
type AxisBounds struct {
	Min float64 `json:"min"`
	Max float64 `json:"max"`
}

// WorkspaceBounds is the arm-base-frame sample region for generated
// sweep poses. Values are in mm.
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

// generatePose samples one arm-target pose aimed at target. Roll about the
// optical axis is uniform in [-π, π]; without roll variation the AX=XB
// solve is degenerate.
func generatePose(target r3.Vector, bounds WorkspaceBounds, rng *rand.Rand) spatialmath.Pose {
	for {
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
		return spatialmath.NewPose(pos, rot)
	}
}
