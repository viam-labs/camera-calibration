package main

import (
	"github.com/viam-labs/camera-calibration/charuco"

	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: posetracker.API, Model: charuco.Model},
	)
}
