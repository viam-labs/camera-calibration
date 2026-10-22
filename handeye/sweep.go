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

// AbsoluteBounds is the arm-base-frame sample region for generated sweep
// poses, resolved to concrete coordinates. This is the internal
// representation used after any relative→absolute transform.
type AbsoluteBounds struct {
	X AxisBounds `json:"x"`
	Y AxisBounds `json:"y"`
	Z AxisBounds `json:"z"`
}

// WorkspaceBoundsMode selects whether user-provided bounds are interpreted as
// absolute arm-base coordinates or as offsets around the seed's observed board
// center. Prefer relative_to_board when the board can move between runs; it
// stays correct without reconfiguration.
const (
	WorkspaceBoundsModeAbsolute        = "absolute"
	WorkspaceBoundsModeRelativeToBoard = "relative_to_board"
)

// WorkspaceBounds is the user-facing sampling-region config. Exactly one of
// Absolute or Relative must be populated, keyed by Mode. Omitting the whole
// block triggers auto-derivation from the seed board center.
type WorkspaceBounds struct {
	Mode     string          `json:"mode"`
	Absolute *AbsoluteBounds `json:"absolute,omitempty"`
	Relative *AbsoluteBounds `json:"relative_to_board,omitempty"`
}

// IsZero reports whether the user omitted workspace_bounds entirely, meaning
// the module should auto-derive bounds around the observed board.
func (w WorkspaceBounds) IsZero() bool {
	return w.Mode == "" && w.Absolute == nil && w.Relative == nil
}

// validateWorkspaceBounds enforces that Mode matches exactly one payload and
// that each axis has Min < Max. Caller must have already confirmed non-zero.
func validateWorkspaceBounds(w WorkspaceBounds) error {
	switch w.Mode {
	case WorkspaceBoundsModeAbsolute:
		if w.Absolute == nil {
			return fmt.Errorf(`workspace_bounds.mode is %q but workspace_bounds.absolute is missing`, w.Mode)
		}
		if w.Relative != nil {
			return fmt.Errorf(`workspace_bounds.mode is %q; remove workspace_bounds.relative_to_board`, w.Mode)
		}
		return validateAxes("workspace_bounds.absolute", *w.Absolute, true)
	case WorkspaceBoundsModeRelativeToBoard:
		if w.Relative == nil {
			return fmt.Errorf(`workspace_bounds.mode is %q but workspace_bounds.relative_to_board is missing`, w.Mode)
		}
		if w.Absolute != nil {
			return fmt.Errorf(`workspace_bounds.mode is %q; remove workspace_bounds.absolute`, w.Mode)
		}
		return validateAxes("workspace_bounds.relative_to_board", *w.Relative, false)
	case "":
		return fmt.Errorf(`workspace_bounds.mode must be %q or %q (to auto-derive, omit the whole workspace_bounds block)`,
			WorkspaceBoundsModeAbsolute, WorkspaceBoundsModeRelativeToBoard)
	default:
		return fmt.Errorf(`workspace_bounds.mode %q unrecognized; must be %q or %q`,
			w.Mode, WorkspaceBoundsModeAbsolute, WorkspaceBoundsModeRelativeToBoard)
	}
}

func validateAxes(path string, b AbsoluteBounds, isAbsolute bool) error {
	for _, axis := range []struct {
		name string
		ab   AxisBounds
	}{{"x", b.X}, {"y", b.Y}, {"z", b.Z}} {
		if axis.ab.Min >= axis.ab.Max {
			hint := ""
			if isAbsolute {
				hint = "; to auto-derive, omit the whole workspace_bounds block"
			}
			return fmt.Errorf("%s.%s.min (%g) must be < max (%g)%s", path, axis.name, axis.ab.Min, axis.ab.Max, hint)
		}
	}
	return nil
}

// Resolve returns the concrete arm-base-frame bounds. For relative_to_board
// mode, offsets are added to boardCenter. Caller must have validated the
// config first (Mode set and matching payload non-nil).
func (w WorkspaceBounds) Resolve(boardCenter r3.Vector) AbsoluteBounds {
	switch w.Mode {
	case WorkspaceBoundsModeRelativeToBoard:
		r := w.Relative
		return AbsoluteBounds{
			X: AxisBounds{Min: boardCenter.X + r.X.Min, Max: boardCenter.X + r.X.Max},
			Y: AxisBounds{Min: boardCenter.Y + r.Y.Min, Max: boardCenter.Y + r.Y.Max},
			Z: AxisBounds{Min: boardCenter.Z + r.Z.Min, Max: boardCenter.Z + r.Z.Max},
		}
	default:
		return *w.Absolute
	}
}

func deriveDefaultBounds(boardCenter r3.Vector) AbsoluteBounds {
	return AbsoluteBounds{
		X: AxisBounds{Min: boardCenter.X - 200, Max: boardCenter.X + 200},
		Y: AxisBounds{Min: boardCenter.Y - 200, Max: boardCenter.Y + 200},
		Z: AxisBounds{Min: boardCenter.Z + 250, Max: boardCenter.Z + 550},
	}
}

// Symmetric variant for eye-to-hand: the arm wiggles the held target around
// its current pose rather than hovering over a world-fixed board.
func deriveDefaultBoundsSymmetric(center r3.Vector) AbsoluteBounds {
	return AbsoluteBounds{
		X: AxisBounds{Min: center.X - 150, Max: center.X + 150},
		Y: AxisBounds{Min: center.Y - 150, Max: center.Y + 150},
		Z: AxisBounds{Min: center.Z - 150, Max: center.Z + 150},
	}
}

// lookAtRotation builds a rotation whose local `forwardLocal` axis points from
// `from` toward `target`, with an additional `rollRad` rotation about that axis.
func lookAtRotation(from, target r3.Vector, rollRad float64, forwardLocal r3.Vector) (*spatialmath.RotationMatrix, error) {
	desired := target.Sub(from)
	if desired.Norm() < 1e-9 {
		return nil, fmt.Errorf("look-at position coincides with target")
	}
	desired = desired.Normalize()

	fw := forwardLocal
	if fw.Norm() < 1e-9 {
		fw = r3.Vector{Z: 1}
	} else {
		fw = fw.Normalize()
	}

	// Orthonormal basis in local frame around fw: aLocal is the component of
	// local +x perpendicular to fw (fall back to +y if +x is parallel to fw).
	localRef := r3.Vector{X: 1}
	if math.Abs(fw.X) > 0.99 {
		localRef = r3.Vector{Y: 1}
	}
	aLocal := localRef.Sub(fw.Mul(localRef.Dot(fw))).Normalize()
	bLocal := fw.Cross(aLocal)

	// Orthonormal basis in world around desired.
	worldUp := r3.Vector{Z: 1}
	if math.Abs(desired.Dot(worldUp)) > 0.99 {
		worldUp = r3.Vector{X: 1}
	}
	aWorld := worldUp.Cross(desired).Normalize()
	bWorld := desired.Cross(aWorld)

	// Roll about desired.
	c, s := math.Cos(rollRad), math.Sin(rollRad)
	aRolled := aWorld.Mul(c).Add(bWorld.Mul(s))
	bRolled := aWorld.Mul(-s).Add(bWorld.Mul(c))

	// R maps local {aLocal, bLocal, fw} to world {aRolled, bRolled, desired}.
	// R[i][j] = aRolled[i]*aLocal[j] + bRolled[i]*bLocal[j] + desired[i]*fw[j]
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
func generatePose(target r3.Vector, bounds AbsoluteBounds, rng *rand.Rand, forwardLocal r3.Vector) spatialmath.Pose {
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
