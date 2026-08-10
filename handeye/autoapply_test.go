package handeye

import (
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

func TestAutoApplyEnabledDefaultsTrue(t *testing.T) {
	cfg := &Config{}
	test.That(t, cfg.autoApplyEnabled(), test.ShouldBeTrue)
}

func TestAutoApplyEnabledRespectsExplicitFalse(t *testing.T) {
	f := false
	cfg := &Config{AutoApplyResult: &f}
	test.That(t, cfg.autoApplyEnabled(), test.ShouldBeFalse)
}

func TestAutoApplyEnabledRespectsExplicitTrue(t *testing.T) {
	tr := true
	cfg := &Config{AutoApplyResult: &tr}
	test.That(t, cfg.autoApplyEnabled(), test.ShouldBeTrue)
}

func TestPassesResidualThresholdsBelowLimits(t *testing.T) {
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg:    &Config{MaxTranslationResidualMM: 5.0, MaxRotationResidualDeg: 2.0},
	}
	pass, err := h.passesResidualThresholds(map[string]interface{}{
		"translation_residual_mm": 1.5,
		"rotation_residual_deg":   0.5,
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pass, test.ShouldBeTrue)
}

func TestPassesResidualThresholdsOverTranslation(t *testing.T) {
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg:    &Config{MaxTranslationResidualMM: 5.0, MaxRotationResidualDeg: 2.0},
	}
	pass, err := h.passesResidualThresholds(map[string]interface{}{
		"translation_residual_mm": 10.0,
		"rotation_residual_deg":   0.5,
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pass, test.ShouldBeFalse)
}

func TestPassesResidualThresholdsOverRotation(t *testing.T) {
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg:    &Config{MaxTranslationResidualMM: 5.0, MaxRotationResidualDeg: 2.0},
	}
	pass, err := h.passesResidualThresholds(map[string]interface{}{
		"translation_residual_mm": 1.0,
		"rotation_residual_deg":   5.0,
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pass, test.ShouldBeFalse)
}

func TestPassesPoseDiversityAboveThreshold(t *testing.T) {
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg:    &Config{MinPoseDiversityDeg: 30.0},
	}
	pass, err := h.passesPoseDiversityThreshold(map[string]interface{}{"pose_diversity_deg": 96.0})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pass, test.ShouldBeTrue)
}

func TestPassesPoseDiversityBelowThreshold(t *testing.T) {
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg:    &Config{MinPoseDiversityDeg: 30.0},
	}
	pass, err := h.passesPoseDiversityThreshold(map[string]interface{}{"pose_diversity_deg": 12.5})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pass, test.ShouldBeFalse)
}

func TestPassesPoseDiversityMissingFieldErrors(t *testing.T) {
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg:    &Config{MinPoseDiversityDeg: 30.0},
	}
	_, err := h.passesPoseDiversityThreshold(map[string]interface{}{})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "pose_diversity_deg")
}

func TestPassesResidualThresholdsMissingFieldErrors(t *testing.T) {
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg:    &Config{MaxTranslationResidualMM: 5.0, MaxRotationResidualDeg: 2.0},
	}
	_, err := h.passesResidualThresholds(map[string]interface{}{})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "translation_residual_mm")
}

func TestPassesReprojectionBelowThreshold(t *testing.T) {
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg:    &Config{MaxReprojectionErrorPx: 2.0},
	}
	pass, err := h.passesReprojectionThreshold(map[string]interface{}{"mean_station_reprojection_px": 0.6})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pass, test.ShouldBeTrue)
}

func TestPassesReprojectionAboveThreshold(t *testing.T) {
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg:    &Config{MaxReprojectionErrorPx: 2.0},
	}
	pass, err := h.passesReprojectionThreshold(map[string]interface{}{"mean_station_reprojection_px": 3.4})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pass, test.ShouldBeFalse)
}

func TestPassesReprojectionMissingFieldErrors(t *testing.T) {
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg:    &Config{MaxReprojectionErrorPx: 2.0},
	}
	_, err := h.passesReprojectionThreshold(map[string]interface{}{})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "mean_station_reprojection_px")
}

func TestResolveTargetCameraUsesConfigOverride(t *testing.T) {
	h := &handeye{cfg: &Config{TargetCamera: "override-cam", PoseTracker: "pt"}}
	got, err := h.resolveTargetCamera(map[string]interface{}{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldEqual, "override-cam")
}

func TestResolveTargetCameraDerivesFromPoseTracker(t *testing.T) {
	h := &handeye{cfg: &Config{PoseTracker: "my-pt"}}
	robotConfig := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name":       "my-pt",
				"attributes": map[string]interface{}{"camera": "derived-cam"},
			},
		},
	}
	got, err := h.resolveTargetCamera(robotConfig)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldEqual, "derived-cam")
}

func TestFrameFromResult(t *testing.T) {
	trans, orient, err := frameFromResult(map[string]interface{}{
		"frame": map[string]interface{}{
			"translation": map[string]interface{}{"x": 1.0},
			"orientation": map[string]interface{}{"type": "ov_degrees"},
		},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, trans, test.ShouldNotBeNil)
	test.That(t, orient, test.ShouldNotBeNil)
}

func TestFrameFromResultMissingTranslation(t *testing.T) {
	_, _, err := frameFromResult(map[string]interface{}{
		"frame": map[string]interface{}{
			"orientation": map[string]interface{}{"type": "ov_degrees"},
		},
	})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "translation")
}

func TestFrameFromResultMissingOrientation(t *testing.T) {
	_, _, err := frameFromResult(map[string]interface{}{
		"frame": map[string]interface{}{
			"translation": map[string]interface{}{"x": 1.0},
		},
	})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "orientation")
}

func TestDeriveTargetCameraFromPoseTracker(t *testing.T) {
	robotConfig := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name":       "my-charuco",
				"attributes": map[string]interface{}{"camera": "sensing-camera"},
			},
		},
	}
	camera, err := deriveTargetCameraFromPoseTracker(robotConfig, "my-charuco")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, camera, test.ShouldEqual, "sensing-camera")
}

func TestDeriveTargetCameraMissingPoseTracker(t *testing.T) {
	_, err := deriveTargetCameraFromPoseTracker(map[string]interface{}{}, "my-charuco")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, `component "my-charuco" not found`)
}

func TestDeriveTargetCameraMissingCameraAttr(t *testing.T) {
	robotConfig := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name":       "my-charuco",
				"attributes": map[string]interface{}{},
			},
		},
	}
	_, err := deriveTargetCameraFromPoseTracker(robotConfig, "my-charuco")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "attributes.camera not set")
}

func TestSetCameraFrameReplacesExistingFrame(t *testing.T) {
	robotConfig := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name": "cam",
				"frame": map[string]interface{}{
					"translation": map[string]interface{}{"x": 0.0, "y": 0.0, "z": 0.0},
					"orientation": map[string]interface{}{"type": "ov_degrees"},
				},
			},
		},
	}
	newTrans := map[string]interface{}{"x": 10.0, "y": 20.0, "z": 30.0}
	newOrient := map[string]interface{}{"type": "ov_degrees", "value": map[string]interface{}{"th": 90.0}}
	err := setCameraFrame(robotConfig, "cam", newTrans, newOrient)
	test.That(t, err, test.ShouldBeNil)

	cam := robotConfig["components"].([]interface{})[0].(map[string]interface{})
	frame := cam["frame"].(map[string]interface{})
	test.That(t, frame["translation"], test.ShouldResemble, newTrans)
	test.That(t, frame["orientation"], test.ShouldResemble, newOrient)
}

func TestSetCameraFrameCreatesFrameIfMissing(t *testing.T) {
	robotConfig := map[string]interface{}{
		"components": []interface{}{map[string]interface{}{"name": "cam"}},
	}
	trans := map[string]interface{}{"x": 1.0}
	orient := map[string]interface{}{"type": "ov_degrees"}
	err := setCameraFrame(robotConfig, "cam", trans, orient)
	test.That(t, err, test.ShouldBeNil)

	cam := robotConfig["components"].([]interface{})[0].(map[string]interface{})
	frame := cam["frame"].(map[string]interface{})
	test.That(t, frame["translation"], test.ShouldResemble, trans)
	test.That(t, frame["orientation"], test.ShouldResemble, orient)
}

func TestSetCameraFrameMissingCameraErrors(t *testing.T) {
	err := setCameraFrame(map[string]interface{}{}, "cam", nil, nil)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, `component "cam" not found`)
	test.That(t, err.Error(), test.ShouldContainSubstring, "auto_apply_result: false")
}

func TestSetCameraFramePreservesOtherComponentFields(t *testing.T) {
	robotConfig := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name":       "cam",
				"model":      "viam:camera:realsense",
				"api":        "rdk:component:camera",
				"attributes": map[string]interface{}{"sensors": []interface{}{"color", "depth"}},
			},
		},
	}
	err := setCameraFrame(robotConfig, "cam", map[string]interface{}{"x": 1.0}, map[string]interface{}{"type": "ov_degrees"})
	test.That(t, err, test.ShouldBeNil)

	cam := robotConfig["components"].([]interface{})[0].(map[string]interface{})
	test.That(t, cam["model"], test.ShouldEqual, "viam:camera:realsense")
	test.That(t, cam["api"], test.ShouldEqual, "rdk:component:camera")
	test.That(t, cam["attributes"], test.ShouldNotBeNil)
}
