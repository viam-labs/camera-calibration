package handeye

import (
	"context"
	"errors"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

func TestBuildFrameSystemUsesService(t *testing.T) {
	fsSvc := inject.NewFrameSystemService("test")
	fsSvc.FrameSystemConfigFunc = func(_ context.Context) (*framesystem.Config, error) {
		return &framesystem.Config{Parts: nil}, nil
	}
	h := &handeye{
		logger:    logging.NewTestLogger(t),
		cfg:       &Config{},
		fsService: fsSvc,
	}
	fs, err := h.buildFrameSystem(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs, test.ShouldNotBeNil)
}

func TestBuildFrameSystemPropagatesServiceError(t *testing.T) {
	fsSvc := inject.NewFrameSystemService("test")
	fsSvc.FrameSystemConfigFunc = func(_ context.Context) (*framesystem.Config, error) {
		return nil, errors.New("boom")
	}
	h := &handeye{
		logger:    logging.NewTestLogger(t),
		cfg:       &Config{},
		fsService: fsSvc,
	}
	_, err := h.buildFrameSystem(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "build frame system")
	test.That(t, err.Error(), test.ShouldContainSubstring, "boom")
}

func TestBuildFrameSystemAppliesInputRangeOverride(t *testing.T) {
	fsSvc := inject.NewFrameSystemService("test")
	fsSvc.FrameSystemConfigFunc = func(_ context.Context) (*framesystem.Config, error) {
		return &framesystem.Config{Parts: nil}, nil
	}
	h := &handeye{
		logger: logging.NewTestLogger(t),
		cfg: &Config{
			InputRangeOverride: map[string]map[string]referenceframe.Limit{
				"nonexistent-arm": {"0": {Min: -1, Max: 1}},
			},
		},
		fsService: fsSvc,
	}
	_, err := h.buildFrameSystem(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "input_range_override")
}
