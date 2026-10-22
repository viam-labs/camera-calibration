package main

import (
	"testing"

	"go.viam.com/test"
)

func TestDownloadCmdFlagsRegistered(t *testing.T) {
	// MarkFlagRequired on --pass-id and the --downloads-dir default both need
	// to be wired at init time; a missing registration would silently accept
	// an incomplete invocation and surprise the operator at runtime.
	passFlag := downloadCmd.Flags().Lookup("pass-id")
	test.That(t, passFlag, test.ShouldNotBeNil)
	dlFlag := downloadCmd.Flags().Lookup("downloads-dir")
	test.That(t, dlFlag, test.ShouldNotBeNil)
	test.That(t, dlFlag.DefValue, test.ShouldEqual, "./downloads")
	orgFlag := downloadCmd.Flags().Lookup("org-id")
	test.That(t, orgFlag, test.ShouldNotBeNil)
}

func TestSolveCmdFlagsRegistered(t *testing.T) {
	passFlag := solveCmd.Flags().Lookup("pass-id")
	test.That(t, passFlag, test.ShouldNotBeNil)
	methodFlag := solveCmd.Flags().Lookup("method")
	test.That(t, methodFlag, test.ShouldNotBeNil)
	test.That(t, methodFlag.DefValue, test.ShouldEqual, "tsai")
	pyFlag := solveCmd.Flags().Lookup("python-bin")
	test.That(t, pyFlag, test.ShouldNotBeNil)
	scriptFlag := solveCmd.Flags().Lookup("solve-script")
	test.That(t, scriptFlag, test.ShouldNotBeNil)
}

func TestStationIDFromFilename(t *testing.T) {
	tests := []struct {
		in   string
		want int
		ok   bool
	}{
		// Current (no-prefix) form.
		{"station-01.json", 1, true},
		{"station-15.json", 15, true},
		// Legacy timestamp-prefixed form so an old pass still replays.
		{"October_22_2026_15_53_31_station-01.json", 1, true},
		{"garbage.json", 0, false},
		{"station-xx.json", 0, false},
	}
	for _, tt := range tests {
		got, ok := stationIDFromFilename(tt.in)
		test.That(t, ok, test.ShouldEqual, tt.ok)
		test.That(t, got, test.ShouldEqual, tt.want)
	}
}
