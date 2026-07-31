#!/usr/bin/env python3
"""Hand-eye calibration solver. Runnable as a CLI for pyrunner."""

import json
import sys
from pathlib import Path

import cv2
import numpy as np


_METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def solve(stations: list, method: str = "tsai") -> dict:
    """Solve for camera-in-gripper transform (X) via AX=XB hand-eye calibration.

    Args:
        stations: list of {"T_be": {...}, "T_cw": {...}}. Each transform is
            {"translation": [x, y, z], "rvec": [rx, ry, rz]} where rvec is
            the compact axis-angle representation (cv2.Rodrigues convention).
            T_be = gripper-in-base (from arm). T_cw = board-in-camera (from
            pose_tracker).
        method: one of "tsai", "park", "horaud", "andreff", "daniilidis".

    Returns:
        {
            "camera_in_gripper_mm": {
                "translation": [x, y, z],
                "rvec": [rx, ry, rz],
            }
        }

    Raises:
        ValueError: fewer than 3 stations or unknown method.
    """
    if len(stations) < 3:
        raise ValueError(f"need at least 3 stations for hand-eye calibration, got {len(stations)}")
    if method not in _METHODS:
        raise ValueError(f"unknown method {method!r}; must be one of {sorted(_METHODS)}")

    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for station in stations:
        be = station["T_be"]
        cw = station["T_cw"]

        R_be, _ = cv2.Rodrigues(np.array(be["rvec"], dtype=np.float64))
        R_gripper2base.append(R_be)
        t_gripper2base.append(np.array(be["translation"], dtype=np.float64).reshape(3, 1))

        R_cw, _ = cv2.Rodrigues(np.array(cw["rvec"], dtype=np.float64))
        R_target2cam.append(R_cw)
        t_target2cam.append(np.array(cw["translation"], dtype=np.float64).reshape(3, 1))

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=_METHODS[method],
    )

    rvec, _ = cv2.Rodrigues(R_cam2gripper)
    return {
        "camera_in_gripper_mm": {
            "translation": t_cam2gripper.flatten().tolist(),
            "rvec": rvec.flatten().tolist(),
        }
    }


def main() -> None:
    if len(sys.argv) != 2:
        sys.stderr.write("usage: solve.py <input_json_path>\n")
        sys.exit(2)
    data = json.loads(Path(sys.argv[1]).read_text())
    result = solve(data["stations"], data.get("method", "tsai"))
    print(json.dumps(result))


if __name__ == "__main__":
    main()
