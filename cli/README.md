# camera-calibration-cli

Troubleshooting tool for `viam:camera-calibration` passes. Pull per-station data from a calibration run and re-run the solver locally.

## Setup

### Build

```bash
make build-cli
```

Produces `./bin/camera-calibration-cli`.

### Authenticate

The CLI reuses the cached [Viam CLI](https://docs.viam.com/dev/tools/cli/) token.

```bash
viam login
```

### Prerequisite on the handeye service

Downloads are only useful if the calibration run actually wrote per-station data. Set this on the handeye service in the fragment or machine config:

```json
"save_capture_images": true
```

Without it, the pass will have plan-request artifacts tagged with the pass ID but no station images or sidecars.

## Commands

### `download` — pull a pass locally

```bash
./bin/camera-calibration-cli download --pass-id <PASS_ID>
```

Writes to `./downloads/<PASS_ID>/`, paired by station:

```
station-01.jpg      station-01.json
station-02.jpg      station-02.json
...
```

Each `.json` sidecar carries the arm end pose, board-in-camera pose, reprojection error, and the recorded calibration type.

| Flag              | Required | Default         | Notes                                                       |
| ----------------- | -------- | --------------- | ----------------------------------------------------------- |
| `--pass-id`       | yes      | —               | Pass ID to download, e.g. `camera-calibration-1792698803`.  |
| `--downloads-dir` | no       | `./downloads`   | Local folder root to write into.                            |
| `--org-id`        | no       | auto-discover   | Skip the per-org probe when you already know the org.       |
| `--debug`         | no       | `false`         | Verbose logs on the download workers.                       |

### `solve` — replay the solver locally against a downloaded pass

```bash
./bin/camera-calibration-cli solve --pass-id <PASS_ID>
```

Bit-exact against what the live module produced on hardware. Lets you test hypotheses in seconds instead of waiting for 3-minute hardware runs: try different solver methods, pick which captures to feed in, etc.

| Flag                 | Required | Default            | Notes                                                                                   |
| -------------------- | -------- | ------------------ | --------------------------------------------------------------------------------------- |
| `--pass-id`          | yes      | —                  | Pass ID to replay. Must already be downloaded.                                          |
| `--downloads-dir`    | no       | `./downloads`      | Local folder holding the pass dir.                                                      |
| `--method`           | no       | `tsai`             | OpenCV solver: `tsai`, `park`, `horaud`, `andreff`, `daniilidis`.                       |
| `--calibration-type` | no       | inferred           | Override `eye_in_hand` or `eye_to_hand` when the sidecar doesn't carry it (old passes). |
| `--python-bin`       | no       | `.venv/bin/python` | Python interpreter.                                                                     |
| `--solve-script`     | no       | `python/solve.py`  | Path to `solve.py` relative to CWD.                                                     |

## Workflows

### Finding the outlier station in a failed or borderline run

Download, then sort stations by reprojection error:

```bash
./bin/camera-calibration-cli download --pass-id <PASS_ID>
cd ./downloads/<PASS_ID>
for f in *.json; do
  python3 -c "import json; d=json.load(open('$f')); print(f'{d[\"station_id\"]:2d}  reproj={d[\"reprojection_error_px\"]:.3f}')"
done | sort -k2
```

Open the `.jpg` for the station with the worst reprojection. That's the image the detector was looking at when it tripped the gate.

### Comparing solver methods on a pass

```bash
./bin/camera-calibration-cli download --pass-id <PASS_ID>
for m in tsai park horaud andreff daniilidis; do
  echo "=== $m ==="
  ./bin/camera-calibration-cli solve --pass-id <PASS_ID> --method $m
done
```

If one method consistently gives tighter residuals or a better frame across several passes, that's a candidate to adopt for the live module.
