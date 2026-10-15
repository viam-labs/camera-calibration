package handeye

import (
	"math"

	"go.viam.com/rdk/spatialmath"
)

// 1 radian of orientation delta is weighted as ~100mm of linear travel for the
// nearest-neighbor cost. Rough proxy; Lite6 end-effector moves roughly that
// distance when rotated 1 rad at a typical arm reach.
const orientationWeightMM = 100.0

// posePool holds pre-filtered candidate poses and pops the one nearest to the
// arm's current end pose. Dynamic nearest-neighbor adapts naturally when a
// pose fails IK (arm stays put) or succeeds (arm advances).
type posePool struct {
	poses []spatialmath.Pose
}

func (p *posePool) add(pose spatialmath.Pose) {
	p.poses = append(p.poses, pose)
}

func (p *posePool) len() int {
	return len(p.poses)
}

func (p *posePool) popNearest(current spatialmath.Pose) spatialmath.Pose {
	bestIdx := 0
	bestDist := poseTravelCost(current, p.poses[0])
	for i := 1; i < len(p.poses); i++ {
		if d := poseTravelCost(current, p.poses[i]); d < bestDist {
			bestDist = d
			bestIdx = i
		}
	}
	chosen := p.poses[bestIdx]
	p.poses = append(p.poses[:bestIdx], p.poses[bestIdx+1:]...)
	return chosen
}

func poseTravelCost(a, b spatialmath.Pose) float64 {
	diff := spatialmath.PoseBetween(a, b)
	posDist := diff.Point().Norm()
	oriDist := math.Abs(diff.Orientation().AxisAngles().Theta)
	return posDist + orientationWeightMM*oriDist
}
