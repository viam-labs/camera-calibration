package handeye

import (
	"fmt"
	"math"
	"sort"
	"strings"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
)

func logStationOk(logger logging.Logger, stationIdx, attempt int, reproj float64, numDetected, markerCount int, distMM, tiltDeg float64, joints []float64) {
	logger.Infof(
		"handeye: station %d (attempt %d) ok: reproj=%.2fpx, corners=%d, markers=%d, dist=%.0fmm, tilt=%.1f°, joints=%s",
		stationIdx, attempt, reproj, numDetected, markerCount, distMM, tiltDeg, jointsDeg(joints),
	)
}

func logNoDetect(logger logging.Logger, attempt, numDetected, markerCount int, joints []float64) {
	logger.Infof(
		"handeye: attempt %d board not detected: corners=%d, markers=%d, joints=%s",
		attempt, numDetected, markerCount, jointsDeg(joints),
	)
}

func logWorstStations(logger logging.Logger, stations []map[string]interface{}, n int) {
	if len(stations) == 0 {
		return
	}
	type scored struct {
		idx    int
		reproj float64
	}
	scores := make([]scored, len(stations))
	for i, st := range stations {
		r, _ := st["reprojection_error_px"].(float64)
		scores[i] = scored{idx: i + 1, reproj: r}
	}
	sort.Slice(scores, func(i, j int) bool { return scores[i].reproj > scores[j].reproj })
	if n > len(scores) {
		n = len(scores)
	}
	parts := make([]string, n)
	for i := 0; i < n; i++ {
		parts[i] = fmt.Sprintf("#%d=%.2fpx", scores[i].idx, scores[i].reproj)
	}
	logger.Infof("handeye: top-%d worst stations by reprojection: %s", n, strings.Join(parts, ", "))
}

// boardDistanceAndTiltDeg returns distance and the angle between board normal
// and camera -z axis (0° = head-on).
func boardDistanceAndTiltDeg(boardInCamera spatialmath.Pose) (distMM, tiltDeg float64) {
	pt := boardInCamera.Point()
	distMM = math.Sqrt(pt.X*pt.X + pt.Y*pt.Y + pt.Z*pt.Z)
	ov := boardInCamera.Orientation().OrientationVectorRadians()
	cos := -ov.OZ
	if cos > 1 {
		cos = 1
	} else if cos < -1 {
		cos = -1
	}
	tiltDeg = math.Acos(cos) * 180.0 / math.Pi
	return
}

func jointsDeg(radians []float64) string {
	if len(radians) == 0 {
		return "[]"
	}
	parts := make([]string, len(radians))
	for i, r := range radians {
		parts[i] = fmt.Sprintf("%.1f", r*180.0/math.Pi)
	}
	return "[" + strings.Join(parts, ", ") + "]°"
}
