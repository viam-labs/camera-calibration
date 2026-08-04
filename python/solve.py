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
            },
            "residuals": {
                "translation_mm": <float>,
                "rotation_deg": <float>,
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

    trans_residual_mm, rot_residual_deg = _residuals(stations, R_cam2gripper, t_cam2gripper)
    pose_diversity_deg = _pose_diversity(stations)

    rvec, _ = cv2.Rodrigues(R_cam2gripper)
    return {
        "camera_in_gripper_mm": {
            "translation": t_cam2gripper.flatten().tolist(),
            "rvec": rvec.flatten().tolist(),
        },
        "residuals": {
            "translation_mm": trans_residual_mm,
            "rotation_deg": rot_residual_deg,
            "pose_diversity_deg": pose_diversity_deg,
        },
    }


def _pose_diversity(stations: list) -> float:
    rotations = [_hmat(s["T_be"])[:3, :3] for s in stations]
    angles = []
    for i in range(len(rotations)):
        for j in range(i + 1, len(rotations)):
            R_diff = rotations[i].T @ rotations[j]
            cos_theta = float(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))
            angles.append(np.arccos(cos_theta))
    if not angles:
        return 0.0
    return float(np.degrees(np.mean(angles)))


def _hmat(pose: dict) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.array(pose["rvec"], dtype=np.float64))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pose["translation"]
    return T


def _residuals(stations: list, R_cam2gripper: np.ndarray, t_cam2gripper: np.ndarray) -> tuple:
    X = np.eye(4)
    X[:3, :3] = R_cam2gripper
    X[:3, 3] = t_cam2gripper.flatten()

    predictions = [_hmat(s["T_be"]) @ X @ _hmat(s["T_cw"]) for s in stations]
    trans = np.array([p[:3, 3] for p in predictions])
    mean_trans = trans.mean(axis=0)
    trans_residual = float(np.mean(np.linalg.norm(trans - mean_trans, axis=1)))

    rots = [p[:3, :3] for p in predictions]
    R_ref = rots[0]
    angles = []
    for R in rots[1:]:
        R_diff = R_ref.T @ R
        cos_theta = float(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))
        angles.append(np.arccos(cos_theta))
    rot_residual = float(np.degrees(np.mean(angles))) if angles else 0.0

    return trans_residual, rot_residual


def main() -> None:
    if len(sys.argv) != 2:
        sys.stderr.write("usage: solve.py <input_json_path>\n")
        sys.exit(2)
    data = json.loads(Path(sys.argv[1]).read_text())
    result = solve(data["stations"], data.get("method", "tsai"))
    print(json.dumps(result))


if __name__ == "__main__":
    main()
