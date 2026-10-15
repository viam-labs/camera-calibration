package handeye

import (
	"math"

	"go.viam.com/rdk/spatialmath"
)

// 1 radian of orientation delta weighted as ~100mm of linear travel; rough
// scale match for a typical arm reach.
const orientationWeightMM = 100.0

// travelWeight trades sweep speed for calibration diversity. 0 = pure
// farthest-from-already-captured (best calibration quality, worst travel
// time). ∞ = pure nearest-to-current (best travel time, poses cluster). 0.5
// weights diversity twice as heavily as travel.
const travelWeight = 0.5

type posePool struct {
	poses []spatialmath.Pose
}

func (p *posePool) add(pose spatialmath.Pose) { p.poses = append(p.poses, pose) }
func (p *posePool) len() int                  { return len(p.poses) }

// popBest removes and returns the pool entry maximizing
//
//	diversity_from(captured) - travelWeight * travel_cost(current).
//
// First capture (captured empty) collapses to nearest-to-current.
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

// diversityFrom returns the min distance from candidate to any captured pose;
// higher = more novel.
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
