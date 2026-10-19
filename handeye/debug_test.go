package handeye

import (
	"encoding/base64"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestBoardDistanceAndTiltDegHeadOn(t *testing.T) {
	ov := &spatialmath.OrientationVector{OX: 0, OY: 0, OZ: -1, Theta: 0}
	pose := spatialmath.NewPose(r3.Vector{X: 0, Y: 0, Z: 1000}, ov)

	dist, tilt := boardDistanceAndTiltDeg(pose)
	test.That(t, dist, test.ShouldAlmostEqual, 1000.0, 1e-6)
	test.That(t, tilt, test.ShouldAlmostEqual, 0.0, 1e-6)
}

func TestBoardDistanceAndTiltDeg45(t *testing.T) {
	ov := &spatialmath.OrientationVector{OX: math.Sqrt(0.5), OY: 0, OZ: -math.Sqrt(0.5), Theta: 0}
	pose := spatialmath.NewPose(r3.Vector{X: 300, Y: 400, Z: 1200}, ov)

	dist, tilt := boardDistanceAndTiltDeg(pose)
	test.That(t, dist, test.ShouldAlmostEqual, math.Sqrt(300*300+400*400+1200*1200), 1e-6)
	test.That(t, tilt, test.ShouldAlmostEqual, 45.0, 1e-4)
}

func TestResolveDebugImageDirDisabledWhenEmpty(t *testing.T) {
	dir := resolveDebugImageDir(logging.NewTestLogger(t), "", "pass123")
	test.That(t, dir, test.ShouldEqual, "")
}

func TestSaveDebugImageWritesFile(t *testing.T) {
	base := t.TempDir()
	dir := resolveDebugImageDir(logging.NewTestLogger(t), base, "passABC")
	test.That(t, dir, test.ShouldNotEqual, "")

	payload := []byte("fake-jpeg-bytes")
	resp := map[string]interface{}{
		"image_bytes_b64": base64.StdEncoding.EncodeToString(payload),
		"image_mime_type": "image/jpeg",
	}
	saveDebugImage(logging.NewTestLogger(t), dir, resp, 7, "ok")

	entries, err := os.ReadDir(dir)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(entries), test.ShouldEqual, 1)
	test.That(t, strings.HasPrefix(entries[0].Name(), "attempt_007_ok"), test.ShouldBeTrue)
	test.That(t, strings.HasSuffix(entries[0].Name(), ".jpg"), test.ShouldBeTrue)

	saved, err := os.ReadFile(filepath.Join(dir, entries[0].Name()))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, saved, test.ShouldResemble, payload)
}

func TestSaveDebugImageNoopOnMissingBytes(t *testing.T) {
	base := t.TempDir()
	dir := resolveDebugImageDir(logging.NewTestLogger(t), base, "passX")
	saveDebugImage(logging.NewTestLogger(t), dir, map[string]interface{}{}, 1, "ok")
	entries, err := os.ReadDir(dir)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(entries), test.ShouldEqual, 0)
}

func TestExtForMime(t *testing.T) {
	test.That(t, extForMime("image/jpeg"), test.ShouldEqual, ".jpg")
	test.That(t, extForMime("image/jpg"), test.ShouldEqual, ".jpg")
	test.That(t, extForMime("image/png"), test.ShouldEqual, ".png")
	test.That(t, extForMime("image/vnd.viam.dep"), test.ShouldEqual, ".bin")
	test.That(t, extForMime(""), test.ShouldEqual, ".bin")
}
