package handeye

import (
	"context"
	"errors"
	"fmt"
	"os"

	"go.viam.com/rdk/app"
	"go.viam.com/rdk/utils"
)

const (
	gateTranslationResidual = "translation_residual"
	gateRotationResidual    = "rotation_residual"
	gatePoseDiversity       = "pose_diversity"
	gateMeanReprojection    = "mean_reprojection"
)

// Non-nil error = malformed solver output (run failure, not quality failure).
func (h *handeye) evaluateQualityGates(quality map[string]interface{}) ([]string, error) {
	var failed []string

	poseDiv, ok := quality["pose_diversity_deg"].(float64)
	if !ok {
		return nil, errors.New("solve result missing quality.pose_diversity_deg")
	}
	if poseDiv < h.cfg.MinPoseDiversityDeg {
		h.logger.Warnf(
			"handeye: pose diversity (%.2f°) below threshold (%.2f°) — arm rotations too clustered for a well-determined solve",
			poseDiv, h.cfg.MinPoseDiversityDeg,
		)
		failed = append(failed, gatePoseDiversity)
	}

	transResidual, ok := quality["translation_residual_mm"].(float64)
	if !ok {
		return nil, errors.New("solve result missing quality.translation_residual_mm")
	}
	if transResidual > h.cfg.MaxTranslationResidualMM {
		h.logger.Warnf("handeye: translation residual (%.2fmm) exceeds threshold (%.2fmm)",
			transResidual, h.cfg.MaxTranslationResidualMM)
		failed = append(failed, gateTranslationResidual)
	}

	rotResidual, ok := quality["rotation_residual_deg"].(float64)
	if !ok {
		return nil, errors.New("solve result missing quality.rotation_residual_deg")
	}
	if rotResidual > h.cfg.MaxRotationResidualDeg {
		h.logger.Warnf("handeye: rotation residual (%.2f°) exceeds threshold (%.2f°)",
			rotResidual, h.cfg.MaxRotationResidualDeg)
		failed = append(failed, gateRotationResidual)
	}

	meanReproj, ok := quality["mean_station_reprojection_px"].(float64)
	if !ok {
		return nil, errors.New("solve result missing quality.mean_station_reprojection_px")
	}
	if meanReproj > h.cfg.MaxReprojectionErrorPx {
		h.logger.Warnf(
			"handeye: mean station reprojection error (%.2f px) exceeds threshold (%.2f px) — detections look inconsistent, check intrinsics/distortion/focus",
			meanReproj, h.cfg.MaxReprojectionErrorPx,
		)
		failed = append(failed, gateMeanReprojection)
	}

	return failed, nil
}

func stampQualityStatus(quality map[string]interface{}, failedGates []string) {
	if len(failedGates) == 0 {
		quality["status"] = "success"
	} else {
		quality["status"] = "failed"
	}
	// protobuf.Struct can't serialize []string; needs []interface{} for ListValue.
	asAny := make([]interface{}, len(failedGates))
	for i, g := range failedGates {
		asAny[i] = g
	}
	quality["failed_gates"] = asAny
}

func (h *handeye) applyToConfig(ctx context.Context, result map[string]interface{}) error {
	partID := os.Getenv(utils.MachinePartIDEnvVar)
	if partID == "" {
		return fmt.Errorf("%s env var not set", utils.MachinePartIDEnvVar)
	}

	client, err := app.CreateViamClientFromEnvVars(ctx, nil, h.logger)
	if err != nil {
		return fmt.Errorf("create app client: %w", err)
	}
	defer func() { _ = client.Close() }()

	appClient := client.AppClient()
	part, _, err := appClient.GetRobotPart(ctx, partID)
	if err != nil {
		return fmt.Errorf("fetch robot part: %w", err)
	}

	targetCamera, err := h.resolveTargetCamera(part.RobotConfig)
	if err != nil {
		return fmt.Errorf("resolve target camera: %w", err)
	}

	parent, translation, orientation, err := frameFromResult(result)
	if err != nil {
		return err
	}

	if err := setCameraFrame(part.RobotConfig, targetCamera, parent, translation, orientation); err != nil {
		return err
	}

	if _, err := appClient.UpdateRobotPart(ctx, partID, part.Name, part.RobotConfig); err != nil {
		return fmt.Errorf("update robot part: %w", err)
	}
	h.logger.Infof("handeye: auto-applied calibrated frame to camera %q", targetCamera)
	return nil
}

func (h *handeye) resolveTargetCamera(robotConfig map[string]interface{}) (string, error) {
	if h.cfg.TargetCamera != "" {
		return h.cfg.TargetCamera, nil
	}
	return deriveTargetCameraFromPoseTracker(robotConfig, h.cfg.PoseTracker)
}

func frameFromResult(result map[string]interface{}) (parent string, translation, orientation map[string]interface{}, err error) {
	frame, ok := result["frame"].(map[string]interface{})
	if !ok {
		return "", nil, nil, errors.New("solve result missing frame")
	}
	parent, ok = frame["parent"].(string)
	if !ok || parent == "" {
		return "", nil, nil, errors.New("solve result missing frame.parent")
	}
	translation, ok = frame["translation"].(map[string]interface{})
	if !ok {
		return "", nil, nil, errors.New("solve result missing frame.translation")
	}
	orientation, ok = frame["orientation"].(map[string]interface{})
	if !ok {
		return "", nil, nil, errors.New("solve result missing frame.orientation")
	}
	return parent, translation, orientation, nil
}

func qualityFromResult(result map[string]interface{}) (map[string]interface{}, error) {
	quality, ok := result["quality"].(map[string]interface{})
	if !ok {
		return nil, errors.New("solve result missing quality")
	}
	return quality, nil
}

func deriveTargetCameraFromPoseTracker(robotConfig map[string]interface{}, poseTrackerName string) (string, error) {
	pt, err := findComponent(robotConfig, poseTrackerName)
	if err != nil {
		return "", err
	}
	attrs, ok := pt["attributes"].(map[string]interface{})
	if !ok {
		return "", fmt.Errorf("pose_tracker %q has no attributes", poseTrackerName)
	}
	camera, ok := attrs["camera"].(string)
	if !ok || camera == "" {
		return "", fmt.Errorf("pose_tracker %q attributes.camera not set; set target_camera on handeye to override", poseTrackerName)
	}
	return camera, nil
}

func setCameraFrame(robotConfig map[string]interface{}, cameraName, parent string, translation, orientation map[string]interface{}) error {
	cam, err := findComponent(robotConfig, cameraName)
	if err != nil {
		return fmt.Errorf("%w — if the camera lives inside a fragment, set auto_apply_result: false and apply the returned frame manually", err)
	}
	frame, _ := cam["frame"].(map[string]interface{})
	if frame == nil {
		frame = map[string]interface{}{}
		cam["frame"] = frame
	}
	frame["parent"] = parent
	frame["translation"] = translation
	frame["orientation"] = orientation
	return nil
}

func findComponent(robotConfig map[string]interface{}, name string) (map[string]interface{}, error) {
	components, _ := robotConfig["components"].([]interface{})
	for _, c := range components {
		cm, ok := c.(map[string]interface{})
		if !ok {
			continue
		}
		if cm["name"] == name {
			return cm, nil
		}
	}
	return nil, fmt.Errorf("component %q not found in components", name)
}
