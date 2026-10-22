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
