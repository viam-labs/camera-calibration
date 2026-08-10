package handeye

import (
	"context"
	"fmt"
	"os"

	"go.viam.com/rdk/app"
	"go.viam.com/rdk/utils"
)

func (h *handeye) verifyCameraFrameParent(ctx context.Context) error {
	partID := os.Getenv(utils.MachinePartIDEnvVar)
	if partID == "" {
		h.logger.Warnf("handeye: skipping camera frame parent check — %s env var not set (running outside module manager?)", utils.MachinePartIDEnvVar)
		return nil
	}
	client, err := app.CreateViamClientFromEnvVars(ctx, nil, h.logger)
	if err != nil {
		return fmt.Errorf("create app client: %w", err)
	}
	defer func() { _ = client.Close() }()

	part, _, err := client.AppClient().GetRobotPart(ctx, partID)
	if err != nil {
		return fmt.Errorf("fetch robot part: %w", err)
	}
	return h.checkCameraFrameParent(part.RobotConfig)
}

func (h *handeye) checkCameraFrameParent(robotConfig map[string]interface{}) error {
	targetCamera, err := h.resolveTargetCamera(robotConfig)
	if err != nil {
		return fmt.Errorf("resolve target camera: %w", err)
	}
	cam, err := findComponent(robotConfig, targetCamera)
	if err != nil {
		return fmt.Errorf("%w — if the camera lives inside a fragment, set auto_apply_result: false and apply the returned frame manually", err)
	}
	frame, _ := cam["frame"].(map[string]interface{})
	parent, _ := frame["parent"].(string)
	if parent == "" {
		return fmt.Errorf("camera %q has no frame.parent set — set it to %q (the arm), so the calibrated transform is meaningful in the frame system", targetCamera, h.cfg.Arm)
	}
	if parent != h.cfg.Arm {
		return fmt.Errorf("camera %q has frame.parent=%q but hand-eye calibration requires it to be parented directly to the arm (%q); fix the camera's frame.parent before running calibrate", targetCamera, parent, h.cfg.Arm)
	}
	return nil
}
