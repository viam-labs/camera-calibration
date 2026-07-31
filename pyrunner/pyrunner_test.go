package pyrunner

import (
	"context"
	"encoding/json"
	"path/filepath"
	"testing"

	"go.viam.com/test"
)

type pingResponse struct {
	OK  bool `json:"ok"`
	PID int  `json:"pid"`
}

// TestPing verifies the Go->Python invocation path end-to-end: spawn the
// venv Python, run python/ping.py, parse the JSON it prints on stdout.
// Requires `make setup` to have created the repo-root venv.
func TestPing(t *testing.T) {
	// go test runs from the package directory; walk one level up to find
	// the repo-root venv and python/ directory.
	pythonBin, err := filepath.Abs(filepath.Join("..", ".venv", "bin", "python"))
	test.That(t, err, test.ShouldBeNil)

	scriptPath, err := filepath.Abs(filepath.Join("..", "python", "ping.py"))
	test.That(t, err, test.ShouldBeNil)

	stdout, err := Run(context.Background(), pythonBin, scriptPath)
	test.That(t, err, test.ShouldBeNil)

	var resp pingResponse
	err = json.Unmarshal(stdout, &resp)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp.OK, test.ShouldBeTrue)
	test.That(t, resp.PID, test.ShouldBeGreaterThan, 0)
}
