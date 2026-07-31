// Package pyrunner runs Python scripts as subprocesses and captures
// their stdout. Spawn-per-call, not persistent.
//
// SPAWN-PER-CALL ASSUMPTION: spawning a fresh Python subprocess per
// invocation is assumed to add negligible overhead relative to a full
// calibration run (~15s of aggregate spawn overhead against ~5min of
// arm movement). If instrumentation shows this to be wrong, migrate to
// spawning once per calibration run and reusing the process across
// detection/solve calls via stdin/stdout JSON.
package pyrunner

import (
	"bytes"
	"context"
	"fmt"
	"os/exec"
	"path/filepath"
	"time"

	"go.viam.com/rdk/logging"
)

// Run executes scriptPath with the given Python interpreter and args,
// returning captured stdout. Stderr is included in the error on non-zero
// exit. Logs the wall-clock duration at Info level so we can validate the
// spawn-per-call assumption with real numbers. Callers pass the venv's
// interpreter (e.g. .venv/bin/python) for dev; once PyInstaller bundling
// lands, callers can invoke the compiled binary directly and skip this
// wrapper.
func Run(ctx context.Context, logger logging.Logger, pythonBin, scriptPath string, args ...string) ([]byte, error) {
	start := time.Now()
	fullArgs := append([]string{scriptPath}, args...)
	//nolint:gosec // pythonBin and scriptPath are internal, validated by caller.
	cmd := exec.CommandContext(ctx, pythonBin, fullArgs...)
	var stdout, stderr bytes.Buffer
	cmd.Stdout = &stdout
	cmd.Stderr = &stderr
	err := cmd.Run()
	duration := time.Since(start)
	logger.Infof("pyrunner: %s completed in %s", filepath.Base(scriptPath), duration)
	if err != nil {
		return nil, fmt.Errorf("%s %s failed after %s: %w\nstderr: %s", pythonBin, scriptPath, duration, err, stderr.String())
	}
	return stdout.Bytes(), nil
}
