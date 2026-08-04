package handeye

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/test"
)

func TestPersistFilePathSanitizes(t *testing.T) {
	got := persistFilePath("svc/with:special")
	test.That(t, filepath.Base(got), test.ShouldEqual, "handeye-svc-with-special-last-result.json")
}

func TestWriteReadRoundTrip(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "state.json")
	orig := persistedState{
		LastResult: map[string]interface{}{
			"translation":             map[string]interface{}{"x": 1.0, "y": 2.0, "z": 3.0},
			"translation_residual_mm": 0.5,
		},
		StartedAt:   time.Date(2026, 8, 4, 12, 0, 0, 0, time.UTC),
		CompletedAt: time.Date(2026, 8, 4, 12, 3, 0, 0, time.UTC),
	}
	test.That(t, writePersistedState(path, orig), test.ShouldBeNil)

	got, err := readPersistedState(path)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got.LastResult["translation_residual_mm"], test.ShouldEqual, 0.5)
	test.That(t, got.StartedAt.Equal(orig.StartedAt), test.ShouldBeTrue)
	test.That(t, got.CompletedAt.Equal(orig.CompletedAt), test.ShouldBeTrue)
}

func TestReadPersistedStateMissingFile(t *testing.T) {
	_, err := readPersistedState(filepath.Join(t.TempDir(), "nope.json"))
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, os.IsNotExist(err), test.ShouldBeTrue)
}

func TestReadPersistedStateCorruptFile(t *testing.T) {
	path := filepath.Join(t.TempDir(), "bad.json")
	test.That(t, os.WriteFile(path, []byte("not json"), 0o600), test.ShouldBeNil)
	_, err := readPersistedState(path)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "parse")
}

func TestLoadLastResultRestoresProgress(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "state.json")
	saved := persistedState{
		LastResult:  map[string]interface{}{"translation_residual_mm": 0.42},
		StartedAt:   time.Date(2026, 8, 4, 12, 0, 0, 0, time.UTC),
		CompletedAt: time.Date(2026, 8, 4, 12, 3, 0, 0, time.UTC),
	}
	data, err := json.Marshal(saved)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, os.WriteFile(path, data, 0o600), test.ShouldBeNil)

	h := &handeye{
		name:        resource.Name{},
		logger:      logging.NewTestLogger(t),
		progress:    progressState{state: "ready"},
		persistPath: path,
	}
	h.loadLastResult()

	test.That(t, h.lastResult["translation_residual_mm"], test.ShouldEqual, 0.42)
	test.That(t, h.progress.state, test.ShouldEqual, "complete")
	test.That(t, h.progress.startedAt.Equal(saved.StartedAt), test.ShouldBeTrue)
	test.That(t, h.progress.completedAt.Equal(saved.CompletedAt), test.ShouldBeTrue)
}

func TestLoadLastResultMissingFileIsNoOp(t *testing.T) {
	h := &handeye{
		name:        resource.Name{},
		logger:      logging.NewTestLogger(t),
		progress:    progressState{state: "ready"},
		persistPath: filepath.Join(t.TempDir(), "nope.json"),
	}
	h.loadLastResult()
	test.That(t, h.lastResult, test.ShouldBeNil)
	test.That(t, h.progress.state, test.ShouldEqual, "ready")
}

func TestSaveLastResultWritesFile(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "state.json")
	h := &handeye{
		name:        resource.Name{},
		logger:      logging.NewTestLogger(t),
		lastResult:  map[string]interface{}{"translation_residual_mm": 0.9},
		progress:    progressState{state: "complete", completedAt: time.Now()},
		persistPath: path,
	}
	h.saveLastResult()

	got, err := readPersistedState(path)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got.LastResult["translation_residual_mm"], test.ShouldEqual, 0.9)
}

func TestDeletePersistedResultRemovesFile(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "state.json")
	test.That(t, os.WriteFile(path, []byte("{}"), 0o600), test.ShouldBeNil)
	h := &handeye{
		name:        resource.Name{},
		logger:      logging.NewTestLogger(t),
		persistPath: path,
	}
	h.deletePersistedResult()
	_, err := os.Stat(path)
	test.That(t, os.IsNotExist(err), test.ShouldBeTrue)
}

func TestDeletePersistedResultMissingFileIsNoOp(t *testing.T) {
	h := &handeye{
		name:        resource.Name{},
		logger:      logging.NewTestLogger(t),
		persistPath: filepath.Join(t.TempDir(), "nope.json"),
	}
	h.deletePersistedResult()
}
