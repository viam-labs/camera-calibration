package handeye

import (
	"math"

	"go.viam.com/rdk/spatialmath"
)

// a 1-radian rotation moves the arm end about this far, so position and angle contribute at the same scale
const orientationWeightMM = 100.0

// pose diversity is twice as important as travel time
const travelWeight = 0.5

type posePool struct {
	poses []spatialmath.Pose
}

func (p *posePool) add(pose spatialmath.Pose) { p.poses = append(p.poses, pose) }
func (p *posePool) len() int                  { return len(p.poses) }

// first call has no history, so it just picks the nearest pose to where the arm already is
func (p *posePool) popBest(current spatialmath.Pose, captured []spatialmath.Pose) spatialmath.Pose {
	bestIdx := 0
	bestScore := poseScore(p.poses[0], current, captured)
	for i := 1; i < len(p.poses); i++ {
		if s := poseScore(p.poses[i], current, captured); s > bestScore {
			bestScore = s
			bestIdx = i
		}
	}
	chosen := p.poses[bestIdx]
	p.poses = append(p.poses[:bestIdx], p.poses[bestIdx+1:]...)
	return chosen
}

func poseScore(candidate, current spatialmath.Pose, captured []spatialmath.Pose) float64 {
	if len(captured) == 0 {
		return -poseTravelCost(current, candidate)
	}
	return diversityFrom(candidate, captured) - travelWeight*poseTravelCost(current, candidate)
}

func diversityFrom(candidate spatialmath.Pose, captured []spatialmath.Pose) float64 {
	minDist := poseTravelCost(candidate, captured[0])
	for i := 1; i < len(captured); i++ {
		if d := poseTravelCost(candidate, captured[i]); d < minDist {
			minDist = d
		}
	}
	return minDist
}

func poseTravelCost(a, b spatialmath.Pose) float64 {
	diff := spatialmath.PoseBetween(a, b)
	posDist := diff.Point().Norm()
	oriDist := math.Abs(diff.Orientation().AxisAngles().Theta)
	return posDist + orientationWeightMM*oriDist
}
