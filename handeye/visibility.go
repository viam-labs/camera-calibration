package handeye

import (
	"context"
	"errors"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

type visibilityParams struct {
	fx, fy, cx, cy float64
	widthPx        int
	heightPx       int
	boardCorners   []r3.Vector // 5 test points in board frame: 4 outer corners + center
}

func (h *handeye) fetchVisibilityParams(ctx context.Context) (*visibilityParams, error) {
	resp, err := h.tracker.DoCommand(ctx, map[string]interface{}{"visibility_params": map[string]interface{}{}})
	if err != nil {
		return nil, fmt.Errorf("fetch visibility_params from pose_tracker: %w", err)
	}
	readFloat := func(k string) (float64, error) {
		v, ok := resp[k].(float64)
		if !ok {
			return 0, fmt.Errorf("visibility_params missing %q", k)
		}
		return v, nil
	}
	readInt := func(k string) (int, error) {
		v, ok := resp[k].(float64)
		if !ok {
			return 0, fmt.Errorf("visibility_params missing %q", k)
		}
		return int(v), nil
	}
	fx, err := readFloat("fx")
	if err != nil {
		return nil, err
	}
	fy, err := readFloat("fy")
	if err != nil {
		return nil, err
	}
	cx, err := readFloat("cx")
	if err != nil {
		return nil, err
	}
	cy, err := readFloat("cy")
	if err != nil {
		return nil, err
	}
	w, err := readInt("width_px")
	if err != nil {
		return nil, err
	}
	hpx, err := readInt("height_px")
	if err != nil {
		return nil, err
	}
	bw, err := readFloat("board_width_mm")
	if err != nil {
		return nil, err
	}
	bh, err := readFloat("board_height_mm")
	if err != nil {
		return nil, err
	}
	if w <= 0 || hpx <= 0 {
		return nil, errors.New("visibility_params width/height must be positive")
	}
	halfW, halfH := bw/2, bh/2
	return &visibilityParams{
		fx: fx, fy: fy, cx: cx, cy: cy,
		widthPx: w, heightPx: hpx,
		boardCorners: []r3.Vector{
			{X: 0, Y: 0, Z: 0},
			{X: -halfW, Y: -halfH, Z: 0},
			{X: +halfW, Y: -halfH, Z: 0},
			{X: -halfW, Y: +halfH, Z: 0},
			{X: +halfW, Y: +halfH, Z: 0},
		},
	}, nil
}

// boardVisibleAt reports whether at least one of the board test points would
// project inside the camera image at the given candidate pose. Permissive by
// design — ChArUco can detect partial boards, so any point in view is enough.
//
// For eye-in-hand: candidatePose is the arm end goal, boardInWorld is fixed
// (from seed), and the camera is approximated as sitting at the arm end.
// For eye-to-hand: candidatePose is the arm end goal (which carries the
// board), the board frame is approximated as the arm end, and cameraInWorld
// is fixed (from seed estimate).
func (v *visibilityParams) boardVisibleAt(candidatePose, boardInWorld, cameraInWorld spatialmath.Pose) bool {
	worldToCamera := spatialmath.PoseInverse(cameraInWorld)
	for _, cornerInBoard := range v.boardCorners {
		cornerInWorld := spatialmath.Compose(boardInWorld, spatialmath.NewPoseFromPoint(cornerInBoard)).Point()
		cornerInCamera := spatialmath.Compose(worldToCamera, spatialmath.NewPoseFromPoint(cornerInWorld)).Point()
		if cornerInCamera.Z <= 0 {
			continue
		}
		u := v.fx*cornerInCamera.X/cornerInCamera.Z + v.cx
		vv := v.fy*cornerInCamera.Y/cornerInCamera.Z + v.cy
		if u >= 0 && u < float64(v.widthPx) && vv >= 0 && vv < float64(v.heightPx) {
			return true
		}
	}
	return false
}
