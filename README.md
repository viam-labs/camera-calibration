# camera-calibration

**Hand-eye calibration** for arm-mounted cameras on Viam machines. Sweep, detect a board, solve, and **write the calibrated frame back to your camera config - automatically**.

The goal is fast, good-enough calibration. If a few millimeters of error is fine for your use case, the [Quick Start](#quick-start) gets you there in ~5 minutes. If you need sub-mm precision, you'll need to pay more attention to the quality of the results.

## Prerequisites

- Arm component configured on the machine.
- Camera component configured (its `frame` doesn't have to be correct - that's what this module fixes).
- Any obstacles the arm might hit during the sweep modeled as components with geometry.

## Quick Start

### Step 1: Get a calibration board

We suggest a ChArUco board, like the following:

![Recommended ChArUco board](charuco-board.png)

### Step 2: Add the module

In the Viam app, add `viam:camera-calibration` to your machine.

```json
{
  "type": "registry",
  "name": "viam_camera-calibration",
  "module_id": "viam:camera-calibration",
  "version": "latest-with-prerelease"
}
```

### Step 3: Configure the pose_tracker

The following values are for the above board.

```json
{
  "name": "calibration-charuco",
  "api": "rdk:component:pose_tracker",
  "model": "viam:camera-calibration:charuco",
  "attributes": {
    "camera": "your-camera-name",
    "dictionary": "DICT_5X5_100",
    "squares_x": 11,
    "squares_y": 8,
    "square_length_mm": 15,
    "marker_length_mm": 12
  }
}
```

### Step 4: Configure the handeye service

```json
{
  "name": "calibration-handeye",
  "api": "rdk:service:generic",
  "model": "viam:camera-calibration:handeye",
  "attributes": {
    "arm": "your-arm-name",
    "pose_tracker": "calibration-charuco"
  }
}
```

Everything else defaults. For best results, add `input_range_override` to tighten the arm's joint limits to your workspace, especially to prevent your cable being yanked around. **If you have joint limits configured in the motion service, you need to separately apply those here - they will NOT be respected automatically.** 

### Step 5: Position the arm

Manually move the arm so the camera has a clean, close view of the whole board. Board should fill 40-70% of the frame, all corners visible. This becomes the starting pose - the arm returns here after calibrating.

### Step 6: Run calibrate

From the app's Control tab, on `calibration-handeye`, send:

```json
{"calibrate": {}}
```

Takes a few minutes. On success, **your camera's `frame` block in the machine config is updated automatically**.

You can call `Status` on `calibration-handeye` to see progress in real-time.

## Reference
### `viam:camera-calibration:charuco`


| Field              | Required | Default         | Description                                                                              |
| ------------------ | -------- | --------------- | ---------------------------------------------------------------------------------------- |
| `camera`           | Yes      | —               | Name of the arm-mounted camera to detect on.                                             |
| `dictionary`       | Yes      | —               | ArUco dictionary. `DICT_{4,5,6,7}X{4,5,6,7}_{50,100,250,1000}` or `DICT_ARUCO_ORIGINAL`. |
| `squares_x`        | Yes      | —               | Board width in squares.                                                                  |
| `squares_y`        | Yes      | —               | Board height in squares.                                                                 |
| `square_length_mm` | Yes      | —               | Physical square edge length in mm.                                                       |
| `marker_length_mm` | Yes      | —               | Physical marker edge length in mm (< `square_length_mm`).                                |
| `image_source`     | No       | first available | Stream to pull from a multi-stream camera (e.g. `"color"` on RGB+depth).                 |
| `distortion`       | No       | from camera     | Override the camera's distortion coefficients (see note below).                          |

> **On `distortion`**: for cameras that undistort on-device (e.g. Zivid, Ensenso), set to `[0, 0, 0, 0, 0]`. For cameras that publish real distortion coefficients (e.g. Realsense), leave unset.

### `viam:camera-calibration:handeye`


| Field                         | Required | Default | Description                                                                   |
| ----------------------------- | -------- | ------- | ----------------------------------------------------------------------------- |
| `arm`                         | Yes      | —       | Name of the arm.                                                              |
| `pose_tracker`                | Yes      | —       | Name of the pose_tracker (usually the charuco above).                         |
| `num_poses`                   | No       | 20      | Successful captures required before solving.                                  |
| `workspace_bounds`            | No       | auto    | Sample region for calibration poses. Auto-derived from the board position if omitted. |
| `settle_seconds`              | No       | 2.0     | Delay after each arm move before capturing.                                   |
| `input_range_override`        | No       | —       | Tighten arm joint limits (radians). Same schema as the built-in motion service. |
| `auto_apply_result`           | No       | `true`  | Write the calibrated frame back to the camera's config.                       |
| `target_camera`               | No       | derived | Override which camera's `frame` block gets updated.                           |
| `max_translation_residual_mm` | No       | 5.0     | Skip auto-apply if solve residual exceeds this.                               |
| `max_rotation_residual_deg`   | No       | 2.0     | Skip auto-apply if solve residual exceeds this.                               |




### DoCommands

`calibrate` — runs the full pipeline; blocks 2-5 min; returns the transform:

```json
{
  "translation": {"x": -37.6, "y": -75.2, "z": 109.5},
  "orientation": {"type": "ov_degrees", "value": {"x": 0.002, "y": 0.002, "z": 1.0, "th": 1.16}},
  "translation_residual_mm": 0.94,
  "rotation_residual_deg": 0.34,
  "auto_applied": true
}
```

`cancel` — halts a running calibrate.

```json
{"cancel": {}}
```

**Status** (standard RDK, not DoCommand) — poll from a second client during a run:

```json
{
  "state": "capturing",
  "positions_captured": 6,
  "positions_required": 20,
  "positions_attempted": 8,
  "elapsed_time": "1m 12s"
}
```

States: `ready | capturing | solving | applying | complete | failed`.
