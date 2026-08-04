package handeye

import (
	"context"
	"errors"
	"fmt"
	"os"

	"go.viam.com/rdk/app"
	"go.viam.com/rdk/utils"
)

func (h *handeye) autoApply(ctx context.Context, result map[string]interface{}) (bool, error) {
	if pass, err := h.passesPoseDiversityThreshold(result); err != nil {
		return false, err
	} else if !pass {
		return false, nil
	}
	if pass, err := h.passesResidualThresholds(result); err != nil {
		return false, err
	} else if !pass {
		return false, nil
	}
	if pass, err := h.passesReprojectionThreshold(result); err != nil {
		return false, err
	} else if !pass {
		return false, nil
	}

	partID := os.Getenv(utils.MachinePartIDEnvVar)
	if partID == "" {
		return false, fmt.Errorf("%s env var not set", utils.MachinePartIDEnvVar)
	}

	client, err := app.CreateViamClientFromEnvVars(ctx, nil, h.logger)
	if err != nil {
		return false, fmt.Errorf("create app client: %w", err)
	}
	defer func() { _ = client.Close() }()

	appClient := client.AppClient()
	part, _, err := appClient.GetRobotPart(ctx, partID)
	if err != nil {
		return false, fmt.Errorf("fetch robot part: %w", err)
	}

	targetCamera, err := h.resolveTargetCamera(part.RobotConfig)
	if err != nil {
		return false, fmt.Errorf("resolve target camera: %w", err)
	}

	translation, orientation, err := frameFromResult(result)
	if err != nil {
		return false, err
	}

	if err := setCameraFrame(part.RobotConfig, targetCamera, translation, orientation); err != nil {
		return false, err
	}

	if _, err := appClient.UpdateRobotPart(ctx, partID, part.Name, part.RobotConfig); err != nil {
		return false, fmt.Errorf("update robot part: %w", err)
	}
	h.logger.Infof("handeye: auto-applied calibrated frame to camera %q", targetCamera)
	return true, nil
}

func (h *handeye) passesPoseDiversityThreshold(result map[string]interface{}) (bool, error) {
	diversity, ok := result["pose_diversity_deg"].(float64)
	if !ok {
		return false, errors.New("solve result missing pose_diversity_deg")
	}
	if diversity < h.cfg.MinPoseDiversityDeg {
		h.logger.Warnf(
			"handeye: pose diversity (%.2f°) below threshold (%.2f°); skipping auto-apply — arm rotations too clustered for a well-determined solve",
			diversity, h.cfg.MinPoseDiversityDeg,
		)
		return false, nil
	}
	return true, nil
}

func (h *handeye) passesResidualThresholds(result map[string]interface{}) (bool, error) {
	transResidual, ok := result["translation_residual_mm"].(float64)
	if !ok {
		return false, errors.New("solve result missing translation_residual_mm")
	}
	rotResidual, ok := result["rotation_residual_deg"].(float64)
	if !ok {
		return false, errors.New("solve result missing rotation_residual_deg")
	}
	if transResidual > h.cfg.MaxTranslationResidualMM || rotResidual > h.cfg.MaxRotationResidualDeg {
		h.logger.Warnf("handeye: residuals (%.2fmm / %.2f°) exceed threshold (%.2fmm / %.2f°); skipping auto-apply",
			transResidual, rotResidual, h.cfg.MaxTranslationResidualMM, h.cfg.MaxRotationResidualDeg)
		return false, nil
	}
	return true, nil
}

func (h *handeye) passesReprojectionThreshold(result map[string]interface{}) (bool, error) {
	meanReproj, ok := result["mean_station_reprojection_px"].(float64)
	if !ok {
		return false, errors.New("solve result missing mean_station_reprojection_px")
	}
	if meanReproj > h.cfg.MaxReprojectionErrorPx {
		h.logger.Warnf(
			"handeye: mean station reprojection error (%.2f px) exceeds threshold (%.2f px); skipping auto-apply — detections look inconsistent, check intrinsics/distortion/focus",
			meanReproj, h.cfg.MaxReprojectionErrorPx,
		)
		return false, nil
	}
	return true, nil
}

func (h *handeye) resolveTargetCamera(robotConfig map[string]interface{}) (string, error) {
	if h.cfg.TargetCamera != "" {
		return h.cfg.TargetCamera, nil
	}
	return deriveTargetCameraFromPoseTracker(robotConfig, h.cfg.PoseTracker)
}

func frameFromResult(result map[string]interface{}) (translation, orientation map[string]interface{}, err error) {
	translation, ok := result["translation"].(map[string]interface{})
	if !ok {
		return nil, nil, errors.New("solve result missing translation")
	}
	orientation, ok = result["orientation"].(map[string]interface{})
	if !ok {
		return nil, nil, errors.New("solve result missing orientation")
	}
	return translation, orientation, nil
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

func setCameraFrame(robotConfig map[string]interface{}, cameraName string, translation, orientation map[string]interface{}) error {
	cam, err := findComponent(robotConfig, cameraName)
	if err != nil {
		return fmt.Errorf("%w; if the camera lives inside a fragment, declare it in components (fragment-only cameras not yet supported by auto_apply_result)", err)
	}
	frame, _ := cam["frame"].(map[string]interface{})
	if frame == nil {
		frame = map[string]interface{}{}
		cam["frame"] = frame
	}
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
