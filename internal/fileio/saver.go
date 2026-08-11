// Package fileio provides an async file saver for motion plan requests.
package fileio

import (
	"context"
	"fmt"
	"path/filepath"
	"time"

	"github.com/erh/vmodutils/file_utils"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"golang.org/x/sync/errgroup"
)

// NewPlanRequestForSaving strips a PlanRequest to the fields we serialize,
// dropping the WorldState (which motion planning may mutate in place).
func NewPlanRequestForSaving(req *armplanning.PlanRequest) *armplanning.PlanRequest {
	return &armplanning.PlanRequest{
		FrameSystem:    req.FrameSystem,
		Goals:          req.Goals,
		StartState:     req.StartState,
		Constraints:    req.Constraints,
		PlannerOptions: req.PlannerOptions,
	}
}

// SaveFile is one unit of async save work.
type SaveFile struct {
	PassID           string                   `json:"pass_id"`
	Filename         string                   `json:"filename"`
	Timestamp        string                   `json:"timestamp"`
	Req              *armplanning.PlanRequest `json:"-"`
	PlanningDuration time.Duration            `json:"planning_duration"`
}

// Filepath returns the full path the SaveFile will be written to,
// namespaced by PassID inside the module's capture dir so Viam data
// manager picks it up.
func (sf SaveFile) Filepath() string {
	return filepath.Join(
		file_utils.GetPathInCaptureDir(fmt.Sprintf("tag=%s", sf.PassID)),
		fmt.Sprintf("%s_%s", sf.Timestamp, sf.Filename),
	)
}

// NewPlanRequestSaveFile builds a SaveFile for a PlanRequest with a
// timestamped filename.
func NewPlanRequestSaveFile(
	req *armplanning.PlanRequest,
	passID string,
	filename string,
	now time.Time,
	planningDuration time.Duration,
) SaveFile {
	return SaveFile{
		PassID:           passID,
		Filename:         filename,
		Timestamp:        now.Format("January_02_2006_15_04_05"),
		Req:              req,
		PlanningDuration: planningDuration,
	}
}

// FileSaver saves files async. If nil, all methods are no-ops.
type FileSaver struct {
	ch         chan SaveFile
	bufferSize int
	ctx        context.Context
	cancel     context.CancelFunc
	g          *errgroup.Group
	logger     logging.Logger
	numWorkers int
}

// NewFileSaver constructs a FileSaver and starts its worker goroutines.
// Callers must Close() (or Drain()) when done to avoid leaking workers.
func NewFileSaver(logger logging.Logger) *FileSaver {
	const bufferSize = 5000
	const numWorkers = 8
	ctx, cancel := context.WithCancel(context.Background())
	s := &FileSaver{
		logger:     logger,
		ch:         make(chan SaveFile, bufferSize),
		bufferSize: bufferSize,
		ctx:        ctx,
		cancel:     cancel,
		g:          new(errgroup.Group),
		numWorkers: numWorkers,
	}
	for i := range numWorkers {
		s.g.Go(func() error {
			s.logger.Infof("FileSaver worker started: %d", i)
			for {
				select {
				case saveFile, ok := <-s.ch:
					if !ok {
						s.logger.Infof("FileSaver is drained: %d", i)
						return nil
					}
					s.logger.Debugf("starting writing %s", saveFile.Filepath())
					if err := file_utils.EnsureDir(filepath.Dir(saveFile.Filepath())); err != nil {
						return err
					}
					if err := saveFile.Req.WriteToFile(saveFile.Filepath()); err != nil {
						s.logger.Warnw("Error saving motion plan",
							"filename", saveFile.Filename, "to", saveFile.Filepath(), "err", err)
					}
					s.logger.Infof("FileSaver wrote %s (planning %s, %d files remaining in queue)",
						saveFile.Filepath(), saveFile.PlanningDuration, len(s.ch))
				case <-s.ctx.Done():
					s.logger.Infof("FileSaver cancelled: %d", i)
					return nil
				}
			}
		})
	}
	return s
}

// SaveAsync enqueues a SaveFile for background writing. Drops the write if
// either the caller's ctx or the saver's own ctx is cancelled.
func (s *FileSaver) SaveAsync(ctx context.Context, sf SaveFile) {
	if s == nil {
		return
	}
	if sf.PassID == "" {
		return
	}

	if ctx.Err() != nil {
		s.logger.Debugf("FileSaver skipping %s: caller ctx cancelled", sf.Filename)
		return
	}
	if s.ctx.Err() != nil {
		s.logger.Warnf("FileSaver skipping %s: saver already cancelled", sf.Filename)
		return
	}

	if len(s.ch) > int(0.8*float64(s.bufferSize)) {
		s.logger.Warnf("FileSaver channel nearly full: %d/%d", len(s.ch), s.bufferSize)
	}

	select {
	case s.ch <- sf:
	case <-ctx.Done():
		s.logger.Debugf("FileSaver dropped %s: caller ctx cancelled while enqueuing", sf.Filename)
	case <-s.ctx.Done():
		s.logger.Debugf("FileSaver dropped %s: saver cancelled while enqueuing", sf.Filename)
	}
}

// Drain closes the channel and waits for all in-flight writes to complete.
// Only safe when no more Save calls will happen.
func (s *FileSaver) Drain() error {
	if s == nil {
		return nil
	}
	s.logger.Infof("FileSaver draining %d files", len(s.ch))
	close(s.ch)
	return s.g.Wait()
}

// Close stops the workers, giving them up to 5 seconds to finish
// draining the queue before cancelling in-flight writes.
func (s *FileSaver) Close() error {
	if s == nil {
		return nil
	}
	if remaining := len(s.ch); remaining > 0 {
		s.logger.Warnf("FileSaver closing with %d files remaining — waiting up to 5s for queue to drain", remaining)
		timer := time.NewTimer(5 * time.Second)
		ticker := time.NewTicker(200 * time.Millisecond)
		defer timer.Stop()
		defer ticker.Stop()
	outer:
		for {
			select {
			case <-timer.C:
				s.logger.Warnf("FileSaver grace period expired with %d files remaining — these will be dropped", len(s.ch))
				break outer
			case <-ticker.C:
				if len(s.ch) == 0 {
					s.logger.Infof("FileSaver queue drained within grace period")
					break outer
				}
			}
		}
	} else {
		s.logger.Infof("FileSaver closing with empty queue")
	}
	s.cancel()
	return s.g.Wait()
}
