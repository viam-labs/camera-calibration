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

// IsZero reports whether every axis is at its zero value (i.e., the
// caller omitted workspace_bounds from config and we should auto-derive).
func (w WorkspaceBounds) IsZero() bool {
	zero := AxisBounds{}
	return w.X == zero && w.Y == zero && w.Z == zero
}

func deriveDefaultBounds(boardCenter r3.Vector) WorkspaceBounds {
	return WorkspaceBounds{
		X: AxisBounds{Min: boardCenter.X - 200, Max: boardCenter.X + 200},
		Y: AxisBounds{Min: boardCenter.Y - 200, Max: boardCenter.Y + 200},
		Z: AxisBounds{Min: boardCenter.Z + 250, Max: boardCenter.Z + 550},
	}
}

// perpBasis returns an orthonormal pair spanning the plane perpendicular to axis.
func perpBasis(axis r3.Vector) (a, b r3.Vector) {
	ref := r3.Vector{Z: 1}
	if math.Abs(axis.Dot(ref)) > 0.99 {
		ref = r3.Vector{X: 1}
	}
	a = ref.Sub(axis.Mul(ref.Dot(axis))).Normalize()
	b = axis.Cross(a)
	return a, b
}

// lookAtRotation builds a rotation whose local `forwardLocal` axis points from
// `from` toward `target`, with an additional `rollRad` rotation about that axis.
func lookAtRotation(from, target r3.Vector, rollRad float64, forwardLocal r3.Vector) (*spatialmath.RotationMatrix, error) {
	desired := target.Sub(from)
	if desired.Norm() < 1e-9 {
		return nil, fmt.Errorf("look-at position coincides with target")
	}
	desired = desired.Normalize()
	fw := forwardLocal.Normalize()

	aLocal, bLocal := perpBasis(fw)
	aWorld, bWorld := perpBasis(desired)

	c, s := math.Cos(rollRad), math.Sin(rollRad)
	aRolled := aWorld.Mul(c).Add(bWorld.Mul(s))
	bRolled := aWorld.Mul(-s).Add(bWorld.Mul(c))

	// Columns are the images of {aLocal, bLocal, fw} under the rotation.
	return spatialmath.NewRotationMatrix([]float64{
		aRolled.X*aLocal.X + bRolled.X*bLocal.X + desired.X*fw.X,
		aRolled.X*aLocal.Y + bRolled.X*bLocal.Y + desired.X*fw.Y,
		aRolled.X*aLocal.Z + bRolled.X*bLocal.Z + desired.X*fw.Z,
		aRolled.Y*aLocal.X + bRolled.Y*bLocal.X + desired.Y*fw.X,
		aRolled.Y*aLocal.Y + bRolled.Y*bLocal.Y + desired.Y*fw.Y,
		aRolled.Y*aLocal.Z + bRolled.Y*bLocal.Z + desired.Y*fw.Z,
		aRolled.Z*aLocal.X + bRolled.Z*bLocal.X + desired.Z*fw.X,
		aRolled.Z*aLocal.Y + bRolled.Z*bLocal.Y + desired.Z*fw.Y,
		aRolled.Z*aLocal.Z + bRolled.Z*bLocal.Z + desired.Z*fw.Z,
	})
}

// generatePose samples one arm-target pose that aims `forwardLocal` at target.
// Roll about that axis is uniform in [-π, π]; without roll variation the AX=XB
// solve is degenerate.
func generatePose(target r3.Vector, bounds WorkspaceBounds, rng *rand.Rand, forwardLocal r3.Vector) spatialmath.Pose {
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
		rot, err := lookAtRotation(pos, target, roll, forwardLocal)
		if err != nil {
			continue
		}
		return spatialmath.NewPose(pos, rot)
	}
}
