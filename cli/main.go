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
	rootCmd       = &cobra.Command{
		Use:   "camera-calibration-cli",
		Short: "CLI for inspecting camera-calibration runs by pass ID",
	}
	downloadCmd = &cobra.Command{
		Use:   "download",
		Short: "Download station images + sidecar JSON for a pass ID into a local folder",
		Run:   runDownload,
	}
)

type DownloadFlags struct {
	PassID             string
	DownloadsDirectory string
	OrgID              string
}

func init() {
	rootCmd.PersistentFlags().BoolVar(&debug, "debug", false, "verbose logging")

	downloadCmd.Flags().StringVar(&downloadFlags.PassID, "pass-id", "", "pass ID to download (required)")
	downloadCmd.Flags().StringVar(&downloadFlags.DownloadsDirectory, "downloads-dir", "./downloads", "local folder to write files into")
	downloadCmd.Flags().StringVar(&downloadFlags.OrgID, "org-id", "", "optional Viam org id to skip auto-discovery")
	_ = downloadCmd.MarkFlagRequired("pass-id")
	rootCmd.AddCommand(downloadCmd)
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
