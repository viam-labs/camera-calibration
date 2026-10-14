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

_CALIBRATION_TYPES = ("eye_in_hand", "eye_to_hand")


def solve(stations: list, method: str = "tsai", calibration_type: str = "eye_in_hand") -> dict:
    """Solve for camera transform (X) via AX=XB hand-eye calibration.

    For eye_in_hand (camera on arm, board in world) X is camera-in-gripper.
    For eye_to_hand (camera static, board on arm) X is camera-in-base; the
    arm poses are inverted before cv2.calibrateHandEye which flips the output
    accordingly.

    Args:
        stations: list of {"T_be": {...}, "T_cw": {...}}. Each transform is
            {"translation": [x, y, z], "rvec": [rx, ry, rz]} where rvec is
            the compact axis-angle representation (cv2.Rodrigues convention).
            T_be = gripper-in-base (from arm). T_cw = board-in-camera (from
            pose_tracker).
        method: one of "tsai", "park", "horaud", "andreff", "daniilidis".
        calibration_type: "eye_in_hand" (default) or "eye_to_hand".

    Returns:
        {
            "camera_in_gripper_mm": {   # historical key; for eye_to_hand this holds camera-in-base
                "translation": [x, y, z],
                "rvec": [rx, ry, rz],
            },
            "residuals": {
                "translation_mm": <float>,
                "rotation_deg": <float>,
            }
        }

    Raises:
        ValueError: fewer than 3 stations, unknown method, or unknown calibration_type.
    """
    if len(stations) < 3:
        raise ValueError(f"need at least 3 stations for hand-eye calibration, got {len(stations)}")
    if method not in _METHODS:
        raise ValueError(f"unknown method {method!r}; must be one of {sorted(_METHODS)}")
    if calibration_type not in _CALIBRATION_TYPES:
        raise ValueError(f"unknown calibration_type {calibration_type!r}; must be one of {list(_CALIBRATION_TYPES)}")

    R_arm = []
    t_arm = []
    R_target2cam = []
    t_target2cam = []

    for station in stations:
        R_be, _ = cv2.Rodrigues(np.array(station["T_be"]["rvec"], dtype=np.float64))
        t_be = np.array(station["T_be"]["translation"], dtype=np.float64).reshape(3, 1)
        if calibration_type == "eye_to_hand":
            R_arm.append(R_be.T)
            t_arm.append(-R_be.T @ t_be)
        else:
            R_arm.append(R_be)
            t_arm.append(t_be)

        R_cw, _ = cv2.Rodrigues(np.array(station["T_cw"]["rvec"], dtype=np.float64))
        R_target2cam.append(R_cw)
        t_target2cam.append(np.array(station["T_cw"]["translation"], dtype=np.float64).reshape(3, 1))

    R_X, t_X = cv2.calibrateHandEye(
        R_arm, t_arm,
        R_target2cam, t_target2cam,
        method=_METHODS[method],
    )

    trans_residual_mm, rot_residual_deg = _residuals(R_arm, t_arm, R_target2cam, t_target2cam, R_X, t_X)
    pose_diversity_deg = _pose_diversity(stations)
    mean_reproj_px, max_reproj_px = _reprojection_stats(stations)

    rvec, _ = cv2.Rodrigues(R_X)
    return {
        "camera_in_gripper_mm": {
            "translation": t_X.flatten().tolist(),
            "rvec": rvec.flatten().tolist(),
        },
        "residuals": {
            "translation_mm": trans_residual_mm,
            "rotation_deg": rot_residual_deg,
            "pose_diversity_deg": pose_diversity_deg,
            "mean_station_reprojection_px": mean_reproj_px,
            "max_station_reprojection_px": max_reproj_px,
        },
    }


def _reprojection_stats(stations: list) -> tuple:
    errors = [float(s["reprojection_error_px"]) for s in stations if "reprojection_error_px" in s]
    if not errors:
        return 0.0, 0.0
    return float(np.mean(errors)), float(np.max(errors))


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


def _residuals(R_arm, t_arm, R_target2cam, t_target2cam, R_X, t_X):
    # T_arm @ X @ T_cw is constant across stations: board-in-base for eye-in-hand,
    # board-in-gripper for eye-to-hand (R_arm is already inverted in that case).
    X = np.eye(4)
    X[:3, :3] = R_X
    X[:3, 3] = t_X.flatten()

    predictions = []
    for R_a, t_a, R_tc, t_tc in zip(R_arm, t_arm, R_target2cam, t_target2cam):
        T_arm = np.eye(4)
        T_arm[:3, :3] = R_a
        T_arm[:3, 3] = t_a.flatten()
        T_tc = np.eye(4)
        T_tc[:3, :3] = R_tc
        T_tc[:3, 3] = t_tc.flatten()
        predictions.append(T_arm @ X @ T_tc)

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
    result = solve(
        data["stations"],
        data.get("method", "tsai"),
        data.get("calibration_type", "eye_in_hand"),
    )
    print(json.dumps(result))


if __name__ == "__main__":
    main()
