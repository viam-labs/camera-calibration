package main

import (
	"github.com/viam-labs/camera-calibration/charuco"
	"github.com/viam-labs/camera-calibration/handeye"

	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: posetracker.API, Model: charuco.Model},
		resource.APIModel{API: generic.API, Model: handeye.Model},
	)
}
