package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"sort"
	"strconv"
	"strings"

	"github.com/spf13/cobra"
	"go.viam.com/rdk/cli"
	"go.viam.com/rdk/logging"
)

func runSolve(cmd *cobra.Command, _ []string) {
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	logger := setupLogger("solve")

	if solveFlags.PassID == "" {
		if usageErr := cmd.Usage(); usageErr != nil {
			logger.Errorf("show usage: %v", usageErr)
		}
		logger.Fatal("--pass-id is required")
	}

	passDir := filepath.Join(solveFlags.DownloadsDirectory, solveFlags.PassID)
	if !passDirHasSidecars(passDir) {
		logger.Infof("pass %s not present locally, downloading first", solveFlags.PassID)
		if err := downloadPassForSolve(ctx, logger); err != nil {
			logger.Fatalf("auto-download: %v", err)
		}
	}

	stations, mode, err := loadStations(passDir)
	if err != nil {
		logger.Fatalf("load stations: %v", err)
	}
	if len(stations) < 3 {
		logger.Fatalf("need at least 3 stations, found %d in %s", len(stations), passDir)
	}

	calibType := solveFlags.CalibrationType
	if calibType == "" {
		calibType = mode
	}
	if calibType == "" {
		logger.Fatal("could not infer calibration_type from sidecars; pass --calibration-type explicitly")
	}

	logger.Infof("loaded %d stations from %s (calibration_type=%s)", len(stations), passDir, calibType)

	result, err := invokeSolver(stations, solveFlags.Method, calibType, solveFlags.PythonBin, solveFlags.SolveScript)
	if err != nil {
		logger.Fatalf("solve: %v", err)
	}

	out, err := json.MarshalIndent(result, "", "  ")
	if err != nil {
		logger.Fatalf("marshal result: %v", err)
	}
	fmt.Println(string(out))
}

// loadStations reads station-NN.json sidecars from passDir and returns the
// stations array in the shape solve.py expects. It also returns the
// calibration_type recorded in the sidecars (empty if not present or mixed).
func loadStations(passDir string) ([]map[string]interface{}, string, error) {
	entries, err := os.ReadDir(passDir)
	if err != nil {
		return nil, "", err
	}
	type idSidecar struct {
		id  int
		doc map[string]interface{}
	}
	var found []idSidecar
	for _, e := range entries {
		if e.IsDir() || !strings.HasSuffix(e.Name(), ".json") {
			continue
		}
		id, ok := stationIDFromFilename(e.Name())
		if !ok {
			continue
		}
		// #nosec G304 - path is a sidecar inside a user-provided passDir; CLI-local use.
		raw, err := os.ReadFile(filepath.Join(passDir, e.Name()))
		if err != nil {
			return nil, "", fmt.Errorf("read %s: %w", e.Name(), err)
		}
		var doc map[string]interface{}
		if err := json.Unmarshal(raw, &doc); err != nil {
			return nil, "", fmt.Errorf("parse %s: %w", e.Name(), err)
		}
		found = append(found, idSidecar{id: id, doc: doc})
	}
	sort.Slice(found, func(i, j int) bool { return found[i].id < found[j].id })

	var stations []map[string]interface{}
	var mode string
	for _, s := range found {
		tBE, ok := s.doc["arm_end_pose"].(map[string]interface{})
		if !ok {
			return nil, "", fmt.Errorf("station %d: missing arm_end_pose", s.id)
		}
		tCW, ok := s.doc["board_in_camera"].(map[string]interface{})
		if !ok {
			return nil, "", fmt.Errorf("station %d: missing board_in_camera", s.id)
		}
		station := map[string]interface{}{
			"T_be": tBE,
			"T_cw": tCW,
		}
		if reproj, ok := s.doc["reprojection_error_px"].(float64); ok {
			station["reprojection_error_px"] = reproj
		}
		stations = append(stations, station)
		if m, ok := s.doc["calibration_type"].(string); ok && m != "" {
			if mode != "" && mode != m {
				return nil, "", fmt.Errorf("mixed calibration_type across sidecars: %q and %q", mode, m)
			}
			mode = m
		}
	}
	return stations, mode, nil
}

func stationIDFromFilename(name string) (int, bool) {
	base := strings.TrimSuffix(name, ".json")
	// Match "station-NN" either as the whole name or at the end (handles the
	// pre-fix-108 timestamp-prefixed sidecars like
	// October_22_2026_15_53_31_station-01.json so an old pass still replays.)
	idx := strings.LastIndex(base, "station-")
	if idx < 0 {
		return 0, false
	}
	id, err := strconv.Atoi(base[idx+len("station-"):])
	if err != nil {
		return 0, false
	}
	return id, true
}

func invokeSolver(stations []map[string]interface{}, method, calibType, pythonBin, solveScript string) (map[string]interface{}, error) {
	input := map[string]interface{}{
		"stations":         stations,
		"method":           method,
		"calibration_type": calibType,
	}
	payload, err := json.Marshal(input)
	if err != nil {
		return nil, fmt.Errorf("marshal input: %w", err)
	}
	tmp, err := os.CreateTemp("", "handeye-solve-*.json")
	if err != nil {
		return nil, fmt.Errorf("create temp file: %w", err)
	}
	defer func() { _ = os.Remove(tmp.Name()) }()
	if _, werr := tmp.Write(payload); werr != nil {
		_ = tmp.Close()
		return nil, fmt.Errorf("write temp file: %w", werr)
	}
	if cerr := tmp.Close(); cerr != nil {
		return nil, fmt.Errorf("close temp file: %w", cerr)
	}

	// #nosec G204 - pythonBin + solveScript are explicit flags with sane defaults; CLI-local use.
	cmd := exec.CommandContext(context.Background(), pythonBin, solveScript, tmp.Name())
	stdout, err := cmd.Output()
	if err != nil {
		var exitErr *exec.ExitError
		if errors.As(err, &exitErr) {
			return nil, fmt.Errorf("solve.py exited %d: %s", exitErr.ExitCode(), string(exitErr.Stderr))
		}
		return nil, fmt.Errorf("run solve.py: %w", err)
	}

	var result map[string]interface{}
	if err := json.Unmarshal(stdout, &result); err != nil {
		return nil, fmt.Errorf("parse solve.py output: %w", err)
	}
	return result, nil
}

// passDirHasSidecars is true if the dir exists and contains at least one
// station-NN.json file. Lets `solve` auto-download when the pass isn't
// present yet without re-downloading already-complete ones.
func passDirHasSidecars(passDir string) bool {
	entries, err := os.ReadDir(passDir)
	if err != nil {
		return false
	}
	for _, e := range entries {
		if e.IsDir() || !strings.HasSuffix(e.Name(), ".json") {
			continue
		}
		if _, ok := stationIDFromFilename(e.Name()); ok {
			return true
		}
	}
	return false
}

func downloadPassForSolve(ctx context.Context, logger logging.Logger) error {
	client, err := cli.ConnectToApp(ctx, logger)
	if err != nil {
		return fmt.Errorf("connect to viam app: %w", err)
	}
	orgID := solveFlags.OrgID
	if orgID == "" {
		orgID, err = discoverOrgForPass(ctx, logger, client, solveFlags.PassID)
		if err != nil {
			return fmt.Errorf("discover org: %w", err)
		}
	}
	passDir := filepath.Join(solveFlags.DownloadsDirectory, solveFlags.PassID)
	if mkErr := os.MkdirAll(passDir, 0o750); mkErr != nil {
		return fmt.Errorf("create pass dir: %w", mkErr)
	}
	files, err := listBinaryDataByTag(ctx, logger, client, orgID, solveFlags.PassID)
	if err != nil {
		return fmt.Errorf("list binary data: %w", err)
	}
	if len(files) == 0 {
		return fmt.Errorf("no files found for pass %q in org %q", solveFlags.PassID, orgID)
	}
	return downloadFiles(ctx, logger, client, passDir, files)
}
