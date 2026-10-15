package handeye

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func testVisParams() *visibilityParams {
	// 640x480 image, 525px focal, center principal point, 180x135mm board.
	return &visibilityParams{
		fx: 525, fy: 525, cx: 320, cy: 240,
		widthPx: 640, heightPx: 480,
		boardCorners: []r3.Vector{
			{X: 0, Y: 0, Z: 0},
			{X: -90, Y: -67.5, Z: 0},
			{X: 90, Y: -67.5, Z: 0},
			{X: -90, Y: 67.5, Z: 0},
			{X: 90, Y: 67.5, Z: 0},
		},
	}
}

func TestBoardVisibleAtCameraLooksDirectlyAtCenteredBoard(t *testing.T) {
	v := testVisParams()
	cameraPose := spatialmath.NewZeroPose()                        // camera at origin, +Z forward
	boardPose := spatialmath.NewPoseFromPoint(r3.Vector{Z: 500})   // board 500mm in front
	candidatePose := spatialmath.NewPoseFromPoint(r3.Vector{Z: 0}) // not used in eye-in-hand math here
	test.That(t, v.boardVisibleAt(candidatePose, boardPose, cameraPose), test.ShouldBeTrue)
}

func TestBoardVisibleAtBoardBehindCameraRejected(t *testing.T) {
	v := testVisParams()
	cameraPose := spatialmath.NewZeroPose()
	boardPose := spatialmath.NewPoseFromPoint(r3.Vector{Z: -500}) // behind camera
	test.That(t, v.boardVisibleAt(spatialmath.NewZeroPose(), boardPose, cameraPose), test.ShouldBeFalse)
}

func TestBoardVisibleAtBoardFarOffAxisRejected(t *testing.T) {
	v := testVisParams()
	cameraPose := spatialmath.NewZeroPose()
	// Board 500mm forward but shifted 10m to the side — way outside FOV.
	boardPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 10000, Z: 500})
	test.That(t, v.boardVisibleAt(spatialmath.NewZeroPose(), boardPose, cameraPose), test.ShouldBeFalse)
}

func TestBoardVisibleAtPartialBoardAccepted(t *testing.T) {
	// Place board center well off to the side so center + 2 corners project
	// outside the image, but the near-side 2 corners project inside. Permissive
	// check should still accept.
	v := testVisParams()
	cameraPose := spatialmath.NewZeroPose()
	// Shift ~165mm right at 500mm depth. Center u projects to ~500*fx/500 + cx = ~173+320 = ~493.
	// Only pixels u<640 accepted; center at ~493 is in, but far corner at 90mm further right
	// projects to ~(165+90)*525/500 + 320 = ~267+320 = ~587 (in). Near corner at 90mm less
	// right projects to ~(165-90)*525/500 + 320 = ~79+320 = ~399 (in).
	// So actually center + both corners might all be in. Let me shift further.
	boardPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 280, Z: 500})
	// Center u ≈ 525*280/500 + 320 = 294+320 = 614 (just inside).
	test.That(t, v.boardVisibleAt(spatialmath.NewZeroPose(), boardPose, cameraPose), test.ShouldBeTrue)
}
