// Package main is the camera-calibration-cli.
package main

import (
	"fmt"
	"os"

	"github.com/spf13/cobra"
	"go.viam.com/rdk/logging"
)

var (
	debug         bool
	downloadFlags DownloadFlags
	solveFlags    SolveFlags
	rootCmd       = &cobra.Command{
		Use:   "camera-calibration-cli",
		Short: "CLI for inspecting camera-calibration runs by pass ID",
	}
	downloadCmd = &cobra.Command{
		Use:   "download",
		Short: "Download station images + sidecar JSON for a pass ID into a local folder",
		Run:   runDownload,
	}
	solveCmd = &cobra.Command{
		Use:   "solve",
		Short: "Re-run the hand-eye solver locally against a downloaded pass's sidecars",
		Run:   runSolve,
	}
)

type DownloadFlags struct {
	PassID             string
	DownloadsDirectory string
	OrgID              string
}

type SolveFlags struct {
	PassID             string
	DownloadsDirectory string
	OrgID              string
	Method             string
	CalibrationType    string
	PythonBin          string
	SolveScript        string
}

func init() {
	rootCmd.PersistentFlags().BoolVar(&debug, "debug", false, "verbose logging")

	downloadCmd.Flags().StringVar(&downloadFlags.PassID, "pass-id", "", "pass ID to download (required)")
	downloadCmd.Flags().StringVar(&downloadFlags.DownloadsDirectory, "downloads-dir", "./downloads", "local folder to write files into")
	downloadCmd.Flags().StringVar(&downloadFlags.OrgID, "org-id", "", "optional Viam org id to skip auto-discovery")
	_ = downloadCmd.MarkFlagRequired("pass-id")
	rootCmd.AddCommand(downloadCmd)

	solveCmd.Flags().StringVar(&solveFlags.PassID, "pass-id", "", "pass ID to replay (required)")
	solveCmd.Flags().StringVar(&solveFlags.DownloadsDirectory, "downloads-dir", "./downloads", "local folder holding the pass dir (will download here if the pass isn't already present)")
	solveCmd.Flags().StringVar(&solveFlags.OrgID, "org-id", "", "optional Viam org id when solve needs to auto-download the pass")
	solveCmd.Flags().StringVar(&solveFlags.Method, "method", "tsai", "solver method: tsai, park, horaud, andreff, daniilidis")
	solveCmd.Flags().StringVar(&solveFlags.CalibrationType, "calibration-type", "", "override calibration_type (eye_in_hand or eye_to_hand); default is inferred from sidecars")
	solveCmd.Flags().StringVar(&solveFlags.PythonBin, "python-bin", ".venv/bin/python", "python interpreter to invoke")
	solveCmd.Flags().StringVar(&solveFlags.SolveScript, "solve-script", "python/solve.py", "path to solve.py relative to CWD")
	_ = solveCmd.MarkFlagRequired("pass-id")
	rootCmd.AddCommand(solveCmd)
}

func main() {
	if err := rootCmd.Execute(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func setupLogger(name string) logging.Logger {
	logger := logging.NewBlankLogger(name)
	logger.SetLevel(logging.INFO)
	logger.AddAppender(logging.NewWriterAppender(os.Stderr))
	if debug {
		logger.SetLevel(logging.DEBUG)
	}
	return logger
}
