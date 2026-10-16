// Package discovery implements a Viam Discovery service that auto-generates
// a charuco pose_tracker config by inspecting a depth camera pointed at a
// printed ChArUco board.
package discovery

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"image"
	"image/png"
	"os"
	"path/filepath"
	"strings"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/services/discovery"
	rdkutils "go.viam.com/rdk/utils"

	"github.com/viam-labs/camera-calibration/pyrunner"
)

// Model is the resource model for the charuco-discovery service.
var Model = resource.NewModel("viam", "camera-calibration", "charuco-discovery")

func init() {
	resource.RegisterService(discovery.API, Model,
		resource.Registration[discovery.Service, *Config]{
			Constructor: newDiscovery,
		},
	)
}

// Config is the attribute configuration for the discovery service.
type Config struct {
	Camera string `json:"camera"`
}

// Validate checks required fields and returns implicit dependencies.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Camera == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "camera")
	}
	return []string{cfg.Camera}, nil, nil
}

type service struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name       resource.Name
	logger     logging.Logger
	cfg        *Config
	camera     camera.Camera
	pythonBin  string
	scriptPath string
}

func newDiscovery(
	_ context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (discovery.Service, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	cam, err := camera.FromProvider(deps, cfg.Camera)
	if err != nil {
		return nil, fmt.Errorf("charuco-discovery: get camera dep %q: %w", cfg.Camera, err)
	}
	exePath, err := os.Executable()
	if err != nil {
		return nil, fmt.Errorf("charuco-discovery: resolve executable path: %w", err)
	}
	moduleRoot := filepath.Dir(filepath.Dir(exePath))
	return &service{
		name:       conf.ResourceName(),
		logger:     logger,
		cfg:        cfg,
		camera:     cam,
		pythonBin:  filepath.Join(moduleRoot, ".venv", "bin", "python"),
		scriptPath: filepath.Join(moduleRoot, "python", "discover.py"),
	}, nil
}

func (s *service) Name() resource.Name { return s.name }

func (s *service) DoCommand(_ context.Context, _ map[string]interface{}) (map[string]interface{}, error) {
	return nil, resource.ErrDoUnimplemented
}

func (s *service) Status(_ context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *service) DiscoverResources(ctx context.Context, _ map[string]any) ([]resource.Config, error) {
	rgbBytes, depthBytes, err := s.fetchRGBAndDepth(ctx)
	if err != nil {
		return nil, err
	}
	intrinsicsJSON, err := s.intrinsicsJSON(ctx)
	if err != nil {
		return nil, err
	}

	rgbPath, cleanupRGB, err := writeTemp(rgbBytes, "rgb-*.img")
	if err != nil {
		return nil, err
	}
	defer cleanupRGB()
	depthPath, cleanupDepth, err := writeTemp(depthBytes, "depth-*.img")
	if err != nil {
		return nil, err
	}
	defer cleanupDepth()

	stdout, err := pyrunner.Run(ctx, s.logger, s.pythonBin, s.scriptPath, rgbPath, depthPath, intrinsicsJSON)
	if err != nil {
		var rerr *pyrunner.RunError
		if errors.As(err, &rerr) && rerr.Stderr != "" {
			return nil, errors.New(strings.TrimSpace(rerr.Stderr))
		}
		return nil, fmt.Errorf("charuco-discovery: %w", err)
	}
	var parsed struct {
		Dictionary     string  `json:"dictionary"`
		SquaresX       int     `json:"squares_x"`
		SquaresY       int     `json:"squares_y"`
		SquareLengthMM float64 `json:"square_length_mm"`
		MarkerLengthMM float64 `json:"marker_length_mm"`
	}
	if err := json.Unmarshal(stdout, &parsed); err != nil {
		return nil, fmt.Errorf("charuco-discovery: parse discover.py output: %w", err)
	}

	return []resource.Config{{
		Name:  "charuco-tracker",
		API:   resource.APINamespaceRDK.WithComponentType("pose_tracker"),
		Model: resource.NewModel("viam", "camera-calibration", "charuco"),
		Attributes: rdkutils.AttributeMap{
			"camera":           s.cfg.Camera,
			"dictionary":       parsed.Dictionary,
			"squares_x":        parsed.SquaresX,
			"squares_y":        parsed.SquaresY,
			"square_length_mm": parsed.SquareLengthMM,
			"marker_length_mm": parsed.MarkerLengthMM,
		},
		ImplicitDependsOn: []string{s.cfg.Camera},
	}}, nil
}

func (s *service) fetchRGBAndDepth(ctx context.Context) (rgb, depth []byte, err error) {
	images, _, err := s.camera.Images(ctx, nil, nil)
	if err != nil {
		return nil, nil, fmt.Errorf("charuco-discovery: camera %q not responding: %w", s.cfg.Camera, err)
	}
	// Decode each stream to image.Image (handles Viam's vnd.viam.dep + rgba formats
	// via rimage's registered decoders) then re-encode as PNG so OpenCV can read it.
	for i := range images {
		img, imgErr := images[i].Image(ctx)
		if imgErr != nil {
			return nil, nil, fmt.Errorf("charuco-discovery: decode image %q: %w", images[i].SourceName, imgErr)
		}
		name := images[i].SourceName
		if name == "depth" {
			s.logger.Infof("charuco-discovery: depth decoded as %T bounds=%v model=%T", img, img.Bounds(), img.ColorModel())
			if dm, ok := img.(*rimage.DepthMap); ok {
				b := dm.Bounds()
				cx, cy := b.Dx()/2, b.Dy()/2
				s.logger.Infof("charuco-discovery: depth sample (center) = %d, (0,0) = %d, (cx,cy-50) = %d",
					dm.GetDepth(cx, cy), dm.GetDepth(0, 0), dm.GetDepth(cx, cy-50))
			}
		}
		pngBytes, pngErr := encodePNG(img)
		if pngErr != nil {
			return nil, nil, fmt.Errorf("charuco-discovery: re-encode %q as PNG: %w", name, pngErr)
		}
		switch {
		case rgb == nil && (name == "color" || name == "" || name == "rgb"):
			rgb = pngBytes
		case depth == nil && name == "depth":
			depth = pngBytes
		}
	}
	if rgb == nil {
		return nil, nil, fmt.Errorf("charuco-discovery: camera %q did not return a color stream", s.cfg.Camera)
	}
	if depth == nil {
		return nil, nil, fmt.Errorf("charuco-discovery: camera %q did not return a depth stream — charuco-discovery requires a depth-capable camera (RealSense, Orbbec, etc.); for RGB-only cameras configure the charuco pose_tracker manually (see README)", s.cfg.Camera)
	}
	return rgb, depth, nil
}

func encodePNG(img image.Image) ([]byte, error) {
	var buf bytes.Buffer
	if err := png.Encode(&buf, img); err != nil {
		return nil, err
	}
	return buf.Bytes(), nil
}

func (s *service) intrinsicsJSON(ctx context.Context) (string, error) {
	props, err := s.camera.Properties(ctx)
	if err != nil {
		return "", fmt.Errorf("charuco-discovery: camera %q not responding: %w", s.cfg.Camera, err)
	}
	if props.IntrinsicParams == nil ||
		props.IntrinsicParams.Fx == 0 || props.IntrinsicParams.Fy == 0 {
		return "", fmt.Errorf("charuco-discovery: camera %q has missing or zero intrinsics — calibrate the camera first", s.cfg.Camera)
	}
	payload := map[string]interface{}{
		"fx": props.IntrinsicParams.Fx,
		"fy": props.IntrinsicParams.Fy,
		"cx": props.IntrinsicParams.Ppx,
		"cy": props.IntrinsicParams.Ppy,
	}
	b, err := json.Marshal(payload)
	if err != nil {
		return "", err
	}
	return string(b), nil
}

func writeTemp(data []byte, pattern string) (string, func(), error) {
	f, err := os.CreateTemp("", pattern)
	if err != nil {
		return "", nil, fmt.Errorf("create temp file: %w", err)
	}
	cleanup := func() { _ = os.Remove(f.Name()) }
	if _, err := f.Write(data); err != nil {
		_ = f.Close()
		cleanup()
		return "", nil, fmt.Errorf("write temp file: %w", err)
	}
	if err := f.Close(); err != nil {
		cleanup()
		return "", nil, fmt.Errorf("close temp file: %w", err)
	}
	return f.Name(), cleanup, nil
}
