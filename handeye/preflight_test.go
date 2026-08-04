package handeye

import (
	"testing"

	"go.viam.com/test"
)

func TestCheckCameraFrameParentAccepts(t *testing.T) {
	h := &handeye{cfg: &Config{Arm: "my-arm", PoseTracker: "pt"}}
	robotConfig := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name":       "pt",
				"attributes": map[string]interface{}{"camera": "cam"},
			},
			map[string]interface{}{
				"name":  "cam",
				"frame": map[string]interface{}{"parent": "my-arm"},
			},
		},
	}
	test.That(t, h.checkCameraFrameParent(robotConfig), test.ShouldBeNil)
}

func TestCheckCameraFrameParentRejectsWorld(t *testing.T) {
	h := &handeye{cfg: &Config{Arm: "my-arm", PoseTracker: "pt"}}
	robotConfig := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name":       "pt",
				"attributes": map[string]interface{}{"camera": "cam"},
			},
			map[string]interface{}{
				"name":  "cam",
				"frame": map[string]interface{}{"parent": "world"},
			},
		},
	}
	err := h.checkCameraFrameParent(robotConfig)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, `frame.parent="world"`)
	test.That(t, err.Error(), test.ShouldContainSubstring, `"my-arm"`)
}

func TestCheckCameraFrameParentRejectsMissing(t *testing.T) {
	h := &handeye{cfg: &Config{Arm: "my-arm", PoseTracker: "pt"}}
	robotConfig := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name":       "pt",
				"attributes": map[string]interface{}{"camera": "cam"},
			},
			map[string]interface{}{
				"name": "cam",
			},
		},
	}
	err := h.checkCameraFrameParent(robotConfig)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no frame.parent set")
}

func TestCheckCameraFrameParentRejectsIntermediateFrame(t *testing.T) {
	h := &handeye{cfg: &Config{Arm: "my-arm", PoseTracker: "pt"}}
	robotConfig := map[string]interface{}{
		"components": []interface{}{
			map[string]interface{}{
				"name":       "pt",
				"attributes": map[string]interface{}{"camera": "cam"},
			},
			map[string]interface{}{
				"name":  "cam",
				"frame": map[string]interface{}{"parent": "custom-bracket"},
			},
		},
	}
	err := h.checkCameraFrameParent(robotConfig)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, `frame.parent="custom-bracket"`)
}
