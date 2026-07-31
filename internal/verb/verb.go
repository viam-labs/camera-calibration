// Package verb provides small helpers for DoCommand verb dispatch shared
// between the module's registered models.
package verb

import "fmt"

// Single returns the single verb key in cmd, or an error if cmd doesn't
// contain exactly one key. Guards against Go's non-deterministic map
// iteration when a caller sends multiple verbs.
func Single(cmd map[string]interface{}) (string, error) {
	if len(cmd) != 1 {
		return "", fmt.Errorf("expected exactly one verb in DoCommand, got %d", len(cmd))
	}
	for verb := range cmd {
		return verb, nil
	}
	return "", nil
}
