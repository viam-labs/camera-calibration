package handeye

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func testVisParams() *visibilityParams {
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
	cameraPose := spatialmath.NewZeroPose()
	boardPose := spatialmath.NewPoseFromPoint(r3.Vector{Z: 500})
	test.That(t, v.boardVisibleAt(boardPose, cameraPose), test.ShouldBeTrue)
}

func TestBoardVisibleAtBoardBehindCameraRejected(t *testing.T) {
	v := testVisParams()
	cameraPose := spatialmath.NewZeroPose()
	boardPose := spatialmath.NewPoseFromPoint(r3.Vector{Z: -500})
	test.That(t, v.boardVisibleAt(boardPose, cameraPose), test.ShouldBeFalse)
}

func TestBoardVisibleAtBoardFarOffAxisRejected(t *testing.T) {
	v := testVisParams()
	cameraPose := spatialmath.NewZeroPose()
	boardPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 10000, Z: 500})
	test.That(t, v.boardVisibleAt(boardPose, cameraPose), test.ShouldBeFalse)
}

func TestBoardVisibleAtPartialBoardAccepted(t *testing.T) {
	v := testVisParams()
	cameraPose := spatialmath.NewZeroPose()
	boardPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 280, Z: 500})
	test.That(t, v.boardVisibleAt(boardPose, cameraPose), test.ShouldBeTrue)
}
