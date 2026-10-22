# camera-calibration-cli

A small command-line tool for pulling station images + sidecar JSON from a calibration run so you can inspect what the detector actually saw at each pose.

## First 5 Minutes: Debugging a Pass

### Step 1: Build the CLI

```bash
make build-cli
```

Produces `./bin/camera-calibration-cli`.

### Step 2: Authenticate

The CLI reuses the cached [Viam CLI](https://docs.viam.com/dev/tools/cli/) token. Log in once:

```bash
viam login
```

### Step 3: Download a Pass

```bash
./bin/camera-calibration-cli download --pass-id <PASS_ID>
```

Files land in `./downloads/<PASS_ID>/`:

```
./downloads/camera-calibration-1792698803/
  station-01.jpg      station-01.json
  station-02.jpg      station-02.json
  ...
  station-15.jpg      station-15.json
```

Each `.json` sidecar carries the arm end pose, the board-in-camera pose, and that station's reprojection error.

**Finding the pass ID**

Either:
- Grab it from the `calibrate` DoCommand result (the sweep logs the pass ID too).
- Grep the machine's logs for `camera-calibration-\d+`.

### Step 4: Find the Outlier Station

```bash
cd ./downloads/<PASS_ID>
for f in *.json; do
  python3 -c "import json; d=json.load(open('$f')); print(f'{d[\"station_id\"]:2d}  reproj={d[\"reprojection_error_px\"]:.3f}')"
done | sort -k2
```

Open the `.jpg` for the station with the worst reprojection. That's the image the detector was looking at when it tripped the gate.

## Flags

| Flag              | Required | Default         | Notes                                                     |
| ----------------- | -------- | --------------- | --------------------------------------------------------- |
| `--pass-id`       | yes      | —               | Pass ID to download, e.g. `camera-calibration-1792698803`.|
| `--downloads-dir` | no       | `./downloads`   | Local folder root to write into.                          |
| `--org-id`        | no       | auto-discover   | Skip the per-org probe when you already know the org.     |
| `--debug`         | no       | `false`         | Verbose logs on the download workers.                     |

## Prerequisite: Enable Saving

The CLI only works if the calibration run actually wrote per-station data. Set this on the handeye service:

```json
"save_capture_images": true
```

Without it, the pass will have plan-request artifacts tagged with the pass ID but no station images or sidecars.
