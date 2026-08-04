package handeye

import (
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"
)

type persistedState struct {
	LastResult  map[string]interface{} `json:"last_result"`
	StartedAt   time.Time              `json:"started_at"`
	CompletedAt time.Time              `json:"completed_at"`
}

var moduleDataDir = func() string {
	if d := os.Getenv("VIAM_MODULE_DATA"); d != "" {
		return d
	}
	return "."
}()

func persistFilePath(resourceName string) string {
	sanitized := strings.NewReplacer("/", "-", ":", "-").Replace(resourceName)
	return filepath.Join(moduleDataDir, fmt.Sprintf("handeye-%s-last-result.json", sanitized))
}

func writePersistedState(path string, state persistedState) error {
	data, err := json.Marshal(state)
	if err != nil {
		return fmt.Errorf("marshal: %w", err)
	}
	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, data, 0o600); err != nil {
		return fmt.Errorf("write tmp: %w", err)
	}
	if err := os.Rename(tmp, path); err != nil {
		_ = os.Remove(tmp)
		return fmt.Errorf("rename: %w", err)
	}
	return nil
}

func readPersistedState(path string) (persistedState, error) {
	var state persistedState
	data, err := os.ReadFile(path) //nolint:gosec // path is derived from module root, not user input
	if err != nil {
		return state, err
	}
	if err := json.Unmarshal(data, &state); err != nil {
		return state, fmt.Errorf("parse: %w", err)
	}
	return state, nil
}

func (h *handeye) saveLastResult() {
	if h.persistPath == "" {
		return
	}
	h.mu.Lock()
	state := persistedState{
		LastResult:  h.lastResult,
		StartedAt:   h.progress.startedAt,
		CompletedAt: h.progress.completedAt,
	}
	h.mu.Unlock()
	if err := writePersistedState(h.persistPath, state); err != nil {
		h.logger.Warnf("handeye: failed to persist last result: %v", err)
	}
}

func (h *handeye) deletePersistedResult() {
	if h.persistPath == "" {
		return
	}
	if err := os.Remove(h.persistPath); err != nil && !errors.Is(err, os.ErrNotExist) {
		h.logger.Warnf("handeye: failed to delete persisted result: %v", err)
	}
}

func (h *handeye) loadLastResult() {
	if h.persistPath == "" {
		return
	}
	state, err := readPersistedState(h.persistPath)
	if err != nil {
		if !errors.Is(err, os.ErrNotExist) {
			h.logger.Warnf("handeye: failed to read persisted result: %v", err)
		}
		return
	}
	if state.LastResult == nil {
		return
	}
	h.mu.Lock()
	h.lastResult = state.LastResult
	h.progress = progressState{
		state:       "complete",
		startedAt:   state.StartedAt,
		completedAt: state.CompletedAt,
	}
	h.mu.Unlock()
}
