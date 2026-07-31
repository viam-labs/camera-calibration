// Package pyrunner runs Python scripts as subprocesses and captures
// their stdout. Spawn-per-call, not persistent.
package pyrunner

import (
	"bytes"
	"context"
	"fmt"
	"os/exec"
)

// Run executes scriptPath with the given Python interpreter and args,
// returning captured stdout. Stderr is included in the error on non-zero
// exit. Callers pass the venv's interpreter (e.g. .venv/bin/python) for
// dev; once PyInstaller bundling lands, callers can invoke the compiled
// binary directly and skip this wrapper.
func Run(ctx context.Context, pythonBin, scriptPath string, args ...string) ([]byte, error) {
	fullArgs := append([]string{scriptPath}, args...)
	//nolint:gosec // pythonBin and scriptPath are internal, validated by caller.
	cmd := exec.CommandContext(ctx, pythonBin, fullArgs...)
	var stdout, stderr bytes.Buffer
	cmd.Stdout = &stdout
	cmd.Stderr = &stderr
	if err := cmd.Run(); err != nil {
		return nil, fmt.Errorf("%s %s failed: %w\nstderr: %s", pythonBin, scriptPath, err, stderr.String())
	}
	return stdout.Bytes(), nil
}
