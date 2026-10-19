package handeye

import (
	"fmt"
	"math"
	"sort"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
)

type debugCapture struct {
	attempt      int
	stationIdx   int
	status       string
	reproj       float64
	numDetected  int
	markerCount  int
	boardDistMM  float64
	boardTiltDeg float64
	joints       []float64
}

type debugSink struct {
	logger logging.Logger
}

func (s *debugSink) logCapture(c debugCapture) {
	if s == nil {
		return
	}
	switch c.status {
	case "ok":
		s.logger.Infof(
			"handeye: station %d (attempt %d) ok: reproj=%.2fpx, corners=%d, markers=%d, dist=%.0fmm, tilt=%.1f°, joints=%s",
			c.stationIdx, c.attempt, c.reproj, c.numDetected, c.markerCount, c.boardDistMM, c.boardTiltDeg, formatJointsDeg(c.joints),
		)
	case "nodetect":
		s.logger.Infof(
			"handeye: attempt %d board not detected: corners=%d, markers=%d, joints=%s",
			c.attempt, c.numDetected, c.markerCount, formatJointsDeg(c.joints),
		)
	}
}

func (s *debugSink) logWorstStations(stations []map[string]interface{}, n int) {
	if s == nil || len(stations) == 0 {
		return
	}
	type scored struct {
		idx    int
		reproj float64
	}
	scores := make([]scored, 0, len(stations))
	for i, st := range stations {
		r, _ := st["reprojection_error_px"].(float64)
		scores = append(scores, scored{idx: i + 1, reproj: r})
	}
	sort.Slice(scores, func(i, j int) bool { return scores[i].reproj > scores[j].reproj })
	if n > len(scores) {
		n = len(scores)
	}
	parts := make([]string, 0, n)
	for i := 0; i < n; i++ {
		parts = append(parts, fmt.Sprintf("#%d=%.2fpx", scores[i].idx, scores[i].reproj))
	}
	s.logger.Infof("handeye: top-%d worst stations by reprojection: %s", n, joinWithComma(parts))
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

func detectCounts(detectResp map[string]interface{}) (numDetected, markerCount int) {
	if v, ok := detectResp["num_detected"].(float64); ok {
		numDetected = int(v)
	}
	if v, ok := detectResp["marker_count"].(float64); ok {
		markerCount = int(v)
	}
	return
}

func formatJointsDeg(radians []float64) string {
	if len(radians) == 0 {
		return "[]"
	}
	parts := make([]string, len(radians))
	for i, r := range radians {
		parts[i] = fmt.Sprintf("%.1f", r*180.0/math.Pi)
	}
	return "[" + joinWithComma(parts) + "]°"
}

func joinWithComma(parts []string) string {
	out := ""
	for i, p := range parts {
		if i > 0 {
			out += ", "
		}
		out += p
	}
	return out
}
