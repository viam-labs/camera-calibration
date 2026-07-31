// Package charuco implements a Viam pose_tracker component that detects
// ChArUco boards and returns each detected corner as a 6-DOF pose in the
// camera frame.
package charuco

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"strconv"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/camera-calibration/internal/verb"
	"github.com/viam-labs/camera-calibration/pyrunner"
)

// Model is the resource model for the charuco pose tracker.
var Model = resource.NewModel("viam", "camera-calibration", "charuco")

func init() {
	resource.RegisterComponent(posetracker.API, Model,
		resource.Registration[posetracker.PoseTracker, *Config]{
			Constructor: newCharuco,
		},
	)
}

// Config is the attribute configuration for the charuco pose tracker.
type Config struct {
	Camera         string  `json:"camera"`
	Dictionary     string  `json:"dictionary"`
	SquaresX       int     `json:"squares_x"`
	SquaresY       int     `json:"squares_y"`
	SquareLengthMM float64 `json:"square_length_mm"`
	MarkerLengthMM float64 `json:"marker_length_mm"`
	// ImageSource optionally selects a specific stream from a multi-stream
	// camera (e.g., "color" on an RGB+depth camera). Empty = take the first
	// stream returned by camera.Images.
	ImageSource string `json:"image_source"`
}

// validDictionaries is the set of ChArUco dictionaries we accept. Names
// match OpenCV's cv2.aruco.DICT_* constants.
var validDictionaries = map[string]struct{}{
	"DICT_4X4_50":         {},
	"DICT_4X4_100":        {},
	"DICT_4X4_250":        {},
	"DICT_4X4_1000":       {},
	"DICT_5X5_50":         {},
	"DICT_5X5_100":        {},
	"DICT_5X5_250":        {},
	"DICT_5X5_1000":       {},
	"DICT_6X6_50":         {},
	"DICT_6X6_100":        {},
	"DICT_6X6_250":        {},
	"DICT_6X6_1000":       {},
	"DICT_7X7_50":         {},
	"DICT_7X7_100":        {},
	"DICT_7X7_250":        {},
	"DICT_7X7_1000":       {},
	"DICT_ARUCO_ORIGINAL": {},
}

// Validate checks that required fields are set and returns the implicit
// dependencies for the component.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Camera == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "camera")
	}
	if cfg.Dictionary == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "dictionary")
	}
	if _, ok := validDictionaries[cfg.Dictionary]; !ok {
		return nil, nil, fmt.Errorf(`invalid dictionary %q; must be one of DICT_{4X4,5X5,6X6,7X7}_{50,100,250,1000} or DICT_ARUCO_ORIGINAL`, cfg.Dictionary)
	}
	if cfg.SquaresX < 2 {
		return nil, nil, fmt.Errorf("squares_x must be >= 2, got %d", cfg.SquaresX)
	}
	if cfg.SquaresY < 2 {
		return nil, nil, fmt.Errorf("squares_y must be >= 2, got %d", cfg.SquaresY)
	}
	if cfg.SquareLengthMM <= 0 {
		return nil, nil, fmt.Errorf("square_length_mm must be > 0, got %g", cfg.SquareLengthMM)
	}
	if cfg.MarkerLengthMM <= 0 {
		return nil, nil, fmt.Errorf("marker_length_mm must be > 0, got %g", cfg.MarkerLengthMM)
	}
	if cfg.MarkerLengthMM >= cfg.SquareLengthMM {
		return nil, nil, fmt.Errorf("marker_length_mm (%g) must be less than square_length_mm (%g)", cfg.MarkerLengthMM, cfg.SquareLengthMM)
	}
	return []string{cfg.Camera}, nil, nil
}

type charuco struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name       resource.Name
	logger     logging.Logger
	cfg        *Config
	camera     camera.Camera
	pythonBin  string
	scriptPath string
}

func newCharuco(
	_ context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (posetracker.PoseTracker, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	cam, err := camera.FromProvider(deps, cfg.Camera)
	if err != nil {
		return nil, fmt.Errorf("charuco: get camera dep %q: %w", cfg.Camera, err)
	}

	// Dev-mode path resolution: anchor to this source file, walk up to repo root.
	// Deployment (PyInstaller) will use a different resolution — separate PR.
	_, thisFile, _, _ := runtime.Caller(0)
	repoRoot := filepath.Dir(filepath.Dir(thisFile))
	return &charuco{
		name:       conf.ResourceName(),
		logger:     logger,
		cfg:        cfg,
		camera:     cam,
		pythonBin:  filepath.Join(repoRoot, ".venv", "bin", "python"),
		scriptPath: filepath.Join(repoRoot, "python", "detect.py"),
	}, nil
}

func (c *charuco) Name() resource.Name {
	return c.name
}

// detectResponse mirrors the JSON produced by python/detect.py.
type detectResponse struct {
	NumDetected int              `json:"num_detected"`
	Corners     []detectedCorner `json:"corners"`
	BoardPoseMM *boardPose       `json:"board_pose_mm"`
}

type detectedCorner struct {
	ID         int       `json:"id"`
	X          float64   `json:"x"`
	Y          float64   `json:"y"`
	PositionMM []float64 `json:"position_mm"`
}

type boardPose struct {
	Translation []float64 `json:"translation"`
	// Rvec is the compact axis-angle rotation from cv2.Rodrigues:
	// magnitude = angle in radians, direction = rotation axis. Converted
	// to a quaternion Go-side via spatialmath.R3ToR4.
	Rvec []float64 `json:"rvec"`
}

func (c *charuco) Poses(
	ctx context.Context,
	_ []string,
	_ map[string]interface{},
) (referenceframe.FrameSystemPoses, error) {
	resp, err := c.captureAndParse(ctx)
	if err != nil {
		return nil, err
	}
	if resp.BoardPoseMM == nil || resp.NumDetected == 0 {
		return referenceframe.FrameSystemPoses{}, nil
	}

	// All corners share the board plane's orientation in the camera frame;
	// translation varies per corner.
	rvec := r3.Vector{
		X: resp.BoardPoseMM.Rvec[0],
		Y: resp.BoardPoseMM.Rvec[1],
		Z: resp.BoardPoseMM.Rvec[2],
	}
	q := spatialmath.Quaternion(spatialmath.R3ToR4(rvec).Quaternion())

	poses := make(referenceframe.FrameSystemPoses, len(resp.Corners))
	for _, corner := range resp.Corners {
		pos := r3.Vector{
			X: corner.PositionMM[0],
			Y: corner.PositionMM[1],
			Z: corner.PositionMM[2],
		}
		name := fmt.Sprintf("corner_%d", corner.ID)
		poses[name] = referenceframe.NewPoseInFrame(c.cfg.Camera, spatialmath.NewPose(pos, &q))
	}
	return poses, nil
}

func (c *charuco) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	v, err := verb.Single(cmd)
	if err != nil {
		return nil, err
	}
	switch v {
	case "detect":
		return c.detectViaCommand(ctx)
	default:
		return nil, fmt.Errorf("unknown verb %q; expected: detect", v)
	}
}

func (c *charuco) detectViaCommand(ctx context.Context) (map[string]interface{}, error) {
	stdout, err := c.captureAndDetect(ctx)
	if err != nil {
		return nil, err
	}
	var result map[string]interface{}
	if err := json.Unmarshal(stdout, &result); err != nil {
		return nil, fmt.Errorf("charuco: parse detect.py output: %w", err)
	}
	return result, nil
}

func (c *charuco) captureAndParse(ctx context.Context) (*detectResponse, error) {
	stdout, err := c.captureAndDetect(ctx)
	if err != nil {
		return nil, err
	}
	var resp detectResponse
	if err := json.Unmarshal(stdout, &resp); err != nil {
		return nil, fmt.Errorf("charuco: parse detect.py output: %w", err)
	}
	return &resp, nil
}

func (c *charuco) captureAndDetect(ctx context.Context) ([]byte, error) {
	intrinsicsJSON, err := c.intrinsicsJSON(ctx)
	if err != nil {
		return nil, err
	}

	var filter []string
	if c.cfg.ImageSource != "" {
		filter = []string{c.cfg.ImageSource}
	}
	images, _, err := c.camera.Images(ctx, filter, nil)
	if err != nil {
		return nil, fmt.Errorf("charuco: capture images: %w", err)
	}
	if len(images) == 0 {
		return nil, errors.New("charuco: camera returned no images")
	}
	imgBytes, err := images[0].Bytes(ctx)
	if err != nil {
		return nil, fmt.Errorf("charuco: get image bytes: %w", err)
	}

	tmpFile, err := os.CreateTemp("", "charuco-*.img")
	if err != nil {
		return nil, fmt.Errorf("charuco: create temp file: %w", err)
	}
	defer func() { _ = os.Remove(tmpFile.Name()) }()

	if _, err := tmpFile.Write(imgBytes); err != nil {
		_ = tmpFile.Close()
		return nil, fmt.Errorf("charuco: write temp file: %w", err)
	}
	if err := tmpFile.Close(); err != nil {
		return nil, fmt.Errorf("charuco: close temp file: %w", err)
	}

	return pyrunner.Run(ctx, c.logger, c.pythonBin, c.scriptPath,
		tmpFile.Name(),
		c.cfg.Dictionary,
		strconv.Itoa(c.cfg.SquaresX),
		strconv.Itoa(c.cfg.SquaresY),
		strconv.FormatFloat(c.cfg.SquareLengthMM, 'f', -1, 64),
		strconv.FormatFloat(c.cfg.MarkerLengthMM, 'f', -1, 64),
		intrinsicsJSON,
	)
}

func (c *charuco) intrinsicsJSON(ctx context.Context) (string, error) {
	props, err := c.camera.Properties(ctx)
	if err != nil {
		return "", fmt.Errorf("charuco: get camera properties: %w", err)
	}
	if props.IntrinsicParams == nil ||
		props.IntrinsicParams.Fx == 0 || props.IntrinsicParams.Fy == 0 ||
		props.IntrinsicParams.Ppx == 0 || props.IntrinsicParams.Ppy == 0 {
		return "", fmt.Errorf("charuco: camera %q has missing or zero intrinsics — calibrate the camera first", c.cfg.Camera)
	}
	if props.DistortionParams == nil {
		return "", fmt.Errorf("charuco: camera %q has no distortion parameters — populate distortion coefficients in the camera config (use zeros if unknown)", c.cfg.Camera)
	}

	payload := map[string]interface{}{
		"fx":         props.IntrinsicParams.Fx,
		"fy":         props.IntrinsicParams.Fy,
		"cx":         props.IntrinsicParams.Ppx,
		"cy":         props.IntrinsicParams.Ppy,
		"distortion": props.DistortionParams.Parameters(),
	}
	buf, err := json.Marshal(payload)
	if err != nil {
		return "", fmt.Errorf("charuco: marshal intrinsics: %w", err)
	}
	return string(buf), nil
}

func (c *charuco) Status(_ context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}
