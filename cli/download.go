package main

import (
	"context"
	"errors"
	"fmt"
	"io"
	"net/http"
	"os"
	"os/signal"
	"path/filepath"

	"github.com/spf13/cobra"
	"go.viam.com/rdk/app"
	"go.viam.com/rdk/cli"
	"go.viam.com/rdk/logging"
	"golang.org/x/sync/errgroup"
)

const (
	binaryDataSearchBatchsize = 1500
	numDownloadWorkers        = 20
)

func runDownload(cmd *cobra.Command, _ []string) {
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	logger := setupLogger("download")

	if downloadFlags.PassID == "" {
		if usageErr := cmd.Usage(); usageErr != nil {
			logger.Errorf("show usage: %v", usageErr)
		}
		logger.Fatal("--pass-id is required")
	}

	client, err := cli.ConnectToApp(ctx, logger)
	if err != nil {
		logger.Fatalf("connect to viam app: %v", err)
	}

	orgID := downloadFlags.OrgID
	if orgID == "" {
		orgID, err = discoverOrgForPass(ctx, logger, client, downloadFlags.PassID)
		if err != nil {
			logger.Fatalf("discover org for pass %q: %v", downloadFlags.PassID, err)
		}
	}

	passDir := filepath.Join(downloadFlags.DownloadsDirectory, downloadFlags.PassID)
	if mkErr := os.MkdirAll(passDir, 0o750); mkErr != nil {
		logger.Fatalf("create pass dir %q: %v", passDir, mkErr)
	}

	files, err := listBinaryDataByTag(ctx, logger, client, orgID, downloadFlags.PassID)
	if err != nil {
		logger.Fatalf("list binary data: %v", err)
	}
	if len(files) == 0 {
		logger.Fatalf("no files found for pass %q in org %q", downloadFlags.PassID, orgID)
	}

	if err := downloadFiles(ctx, logger, client, passDir, files); err != nil && !errors.Is(err, context.Canceled) {
		logger.Fatalf("download: %v", err)
	}
	logger.Infof("downloaded %d files to %s", len(files), passDir)
}

// discoverOrgForPass walks the user's accessible orgs probing BinaryDataByFilter
// until one returns a hit for the pass tag.
func discoverOrgForPass(ctx context.Context, logger logging.Logger, client *app.ViamClient, passID string) (string, error) {
	orgs, err := client.AppClient().ListOrganizations(ctx)
	if err != nil {
		return "", fmt.Errorf("list organizations: %w", err)
	}
	if len(orgs) == 0 {
		return "", errors.New("no organizations found for authenticated user")
	}
	for _, org := range orgs {
		if ctx.Err() != nil {
			return "", ctx.Err()
		}
		resp, err := client.DataClient().BinaryDataByFilter(ctx, false, &app.DataByFilterOptions{
			Filter: &app.Filter{
				OrganizationIDs: []string{org.ID},
				TagsFilter: app.TagsFilter{
					Type: app.TagsFilterTypeMatchByOr,
					Tags: []string{passID},
				},
			},
			Limit: 1,
		})
		if err != nil {
			logger.Debugf("probe org %s (%s): %v", org.Name, org.ID, err)
			continue
		}
		if len(resp.BinaryData) == 0 {
			continue
		}
		logger.Infof("pass %s found in org %s (%s)", passID, org.Name, org.ID)
		return org.ID, nil
	}
	return "", fmt.Errorf("pass %q not found in any accessible organization", passID)
}

func listBinaryDataByTag(ctx context.Context, logger logging.Logger, client *app.ViamClient, orgID, passID string) ([]*app.BinaryData, error) {
	var out []*app.BinaryData
	last := ""
	for {
		resp, err := client.DataClient().BinaryDataByFilter(ctx, false, &app.DataByFilterOptions{
			Filter: &app.Filter{
				OrganizationIDs: []string{orgID},
				TagsFilter: app.TagsFilter{
					Type: app.TagsFilterTypeMatchByOr,
					Tags: []string{passID},
				},
			},
			Last:  last,
			Limit: binaryDataSearchBatchsize,
		})
		if err != nil {
			return nil, err
		}
		logger.Debugf("got %d binary data in batch", len(resp.BinaryData))
		for _, bd := range resp.BinaryData {
			if bd.Metadata == nil || bd.Metadata.FileName == "" {
				continue
			}
			out = append(out, bd)
		}
		if resp.Count < binaryDataSearchBatchsize {
			break
		}
		last = resp.Last
	}
	return out, nil
}

func downloadFiles(ctx context.Context, logger logging.Logger, client *app.ViamClient, passDir string, files []*app.BinaryData) error {
	g, gctx := errgroup.WithContext(ctx)
	ch := make(chan *app.BinaryData, len(files))
	worker := func() error {
		for {
			select {
			case <-gctx.Done():
				return gctx.Err()
			case bd, ok := <-ch:
				if !ok {
					return nil
				}
				dest := filepath.Join(passDir, filepath.Base(bd.Metadata.FileName))
				if info, err := os.Stat(dest); err == nil && !info.IsDir() {
					logger.Debugf("skip %s (already downloaded)", dest)
					continue
				}
				url, err := client.DataClient().CreateBinaryDataSignedURL(gctx, bd.Metadata.BinaryDataID, 20)
				if err != nil {
					return fmt.Errorf("signed url for %s: %w", bd.Metadata.FileName, err)
				}
				if err := fetchToFile(gctx, url, dest); err != nil {
					return fmt.Errorf("fetch %s: %w", bd.Metadata.FileName, err)
				}
				logger.Infof("downloaded %s", filepath.Base(dest))
			}
		}
	}
	for range numDownloadWorkers {
		g.Go(worker)
	}
	for _, bd := range files {
		ch <- bd
	}
	close(ch)
	return g.Wait()
}

func fetchToFile(ctx context.Context, url, dest string) error {
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, url, nil)
	if err != nil {
		return err
	}
	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		return err
	}
	defer func() { _ = resp.Body.Close() }()
	if resp.StatusCode != http.StatusOK {
		return fmt.Errorf("http %d", resp.StatusCode)
	}
	// #nosec G304 - path derived from user-supplied downloads-dir; CLI-local.
	f, err := os.Create(dest)
	if err != nil {
		return err
	}
	defer func() { _ = f.Close() }()
	_, err = io.Copy(f, resp.Body)
	return err
}
