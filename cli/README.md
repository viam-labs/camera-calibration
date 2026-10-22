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

### Step 5: Re-run the Solver Locally (optional)

Replay the hand-eye solve against the downloaded sidecars without touching hardware. Bit-exact against what the live run produced, so you can test hypotheses in seconds instead of waiting for 3-minute calibrations.

```bash
./bin/camera-calibration-cli solve --pass-id <PASS_ID>
```

Prints the resulting frame + residuals. Reads `station-*.json` from `./downloads/<PASS_ID>/`, invokes `python/solve.py` locally, and returns the same JSON the live module would have produced.

If `calibration_type` isn't recorded in the sidecars (older passes from before this landed), pass it explicitly:

```bash
./bin/camera-calibration-cli solve --pass-id <PASS_ID> --calibration-type eye_in_hand
```

## `download` flags

| Flag              | Required | Default         | Notes                                                     |
| ----------------- | -------- | --------------- | --------------------------------------------------------- |
| `--pass-id`       | yes      | —               | Pass ID to download, e.g. `camera-calibration-1792698803`.|
| `--downloads-dir` | no       | `./downloads`   | Local folder root to write into.                          |
| `--org-id`        | no       | auto-discover   | Skip the per-org probe when you already know the org.     |
| `--debug`         | no       | `false`         | Verbose logs on the download workers.                     |

## `solve` flags

| Flag                 | Required | Default            | Notes                                                                                   |
| -------------------- | -------- | ------------------ | --------------------------------------------------------------------------------------- |
| `--pass-id`          | yes      | —                  | Pass ID to replay, e.g. `camera-calibration-1792698803`. Must already be downloaded.    |
| `--downloads-dir`    | no       | `./downloads`      | Local folder holding the pass dir.                                                      |
| `--method`           | no       | `tsai`             | OpenCV solver: `tsai`, `park`, `horaud`, `andreff`, `daniilidis`.                       |
| `--calibration-type` | no       | inferred           | Override `eye_in_hand` or `eye_to_hand` when the sidecar doesn't carry it.              |
| `--python-bin`       | no       | `.venv/bin/python` | Python interpreter.                                                                     |
| `--solve-script`     | no       | `python/solve.py`  | Path to `solve.py` relative to CWD.                                                     |

## Prerequisite: Enable Saving

The CLI only works if the calibration run actually wrote per-station data. Set this on the handeye service:

```json
"save_capture_images": true
```

Without it, the pass will have plan-request artifacts tagged with the pass ID but no station images or sidecars.
