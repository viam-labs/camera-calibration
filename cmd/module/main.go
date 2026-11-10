// Package main runs the camera-calibration Viam module, registering the
// charuco pose tracker and handeye calibration service.
package main

import (
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	discoverysvc "go.viam.com/rdk/services/discovery"
	"go.viam.com/rdk/services/generic"

	"github.com/viam-labs/camera-calibration/charuco"
	"github.com/viam-labs/camera-calibration/discovery"
	"github.com/viam-labs/camera-calibration/handeye"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: posetracker.API, Model: charuco.Model},
		resource.APIModel{API: generic.API, Model: handeye.Model},
		resource.APIModel{API: generic.API, Model: handeye.ModelEyeToHand},
		resource.APIModel{API: generic.API, Model: handeye.DeprecatedModel},
		resource.APIModel{API: discoverysvc.API, Model: discovery.Model},
	)
}
