#!/usr/bin/env python3
"""ChArUco corner detection + PnP. Runnable as a CLI for pyrunner."""

import json
import sys
from pathlib import Path

import cv2
import numpy as np
from cv2 import aruco


_DICTIONARY_NAMES = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_4X4_1000": aruco.DICT_4X4_1000,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_5X5_1000": aruco.DICT_5X5_1000,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_100": aruco.DICT_6X6_100,
    "DICT_6X6_250": aruco.DICT_6X6_250,
    "DICT_6X6_1000": aruco.DICT_6X6_1000,
    "DICT_7X7_50": aruco.DICT_7X7_50,
    "DICT_7X7_100": aruco.DICT_7X7_100,
    "DICT_7X7_250": aruco.DICT_7X7_250,
    "DICT_7X7_1000": aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
}


def detect(
    image_bytes: bytes,
    dictionary: str,
    squares_x: int,
    squares_y: int,
    square_length_mm: float,
    marker_length_mm: float,
    intrinsics: dict,
) -> dict:
    """Detect ChArUco corners and compute board pose via PnP.

    Args:
        image_bytes: raw image bytes (JPEG, PNG, etc.). Decoded via cv2.imdecode.
        dictionary: ArUco dictionary name (e.g., "DICT_5X5_100").
        squares_x, squares_y: board grid dimensions (number of squares).
        square_length_mm: physical chessboard square size in mm.
        marker_length_mm: physical ArUco marker size in mm; must be < square_length_mm.
        intrinsics: {"fx", "fy", "cx", "cy", "distortion": [...]}. Distortion is
            a list of Brown-Conrady coefficients in cv2 order (k1, k2, p1, p2, k3);
            empty list = no distortion.

    Returns:
        {
            "num_detected": int,
            "corners": [{"id": int, "x": float, "y": float, "position_mm": [x, y, z]}, ...],
            "board_pose_mm": {"translation": [x, y, z], "rvec": [rx, ry, rz]} | None,
        }
        `corners` is empty and `board_pose_mm` is None if fewer than 4 corners are
        detected (insufficient for PnP). `rvec` is the compact axis-angle
        representation from cv2.Rodrigues (magnitude = angle, direction = axis);
        Go-side conversion to quaternion via RDK spatialmath.R3ToR4.

    Raises:
        ValueError: unknown dictionary name or unreadable image bytes.
    """
    if dictionary not in _DICTIONARY_NAMES:
        raise ValueError(f"unknown dictionary {dictionary!r}")

    arr = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("failed to decode image bytes")

    aruco_dict = aruco.getPredefinedDictionary(_DICTIONARY_NAMES[dictionary])
    board = aruco.CharucoBoard(
        (squares_x, squares_y),
        square_length_mm,
        marker_length_mm,
        aruco_dict,
    )
    detector = aruco.CharucoDetector(board)

    charuco_corners, charuco_ids, _marker_corners, _marker_ids = detector.detectBoard(image)

    if charuco_corners is None or charuco_ids is None or len(charuco_ids) < 4:
        return {"num_detected": 0, "corners": [], "board_pose_mm": None}

    camera_matrix = np.array([
        [intrinsics["fx"], 0.0, intrinsics["cx"]],
        [0.0, intrinsics["fy"], intrinsics["cy"]],
        [0.0, 0.0, 1.0],
    ])
    dist = np.array(intrinsics.get("distortion") or [0.0, 0.0, 0.0, 0.0, 0.0])

    obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
    if obj_points is None or len(obj_points) < 4:
        return {"num_detected": 0, "corners": [], "board_pose_mm": None}

    ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist)
    if not ok:
        return {"num_detected": 0, "corners": [], "board_pose_mm": None}

    R, _ = cv2.Rodrigues(rvec)
    t = tvec.flatten()
    r = rvec.flatten()

    corners = []
    for i in range(len(charuco_ids)):
        cid = int(charuco_ids[i].item())
        px, py = charuco_corners[i][0]
        board_pt = obj_points[i][0]
        cam_pt = R @ board_pt + t
        corners.append({
            "id": cid,
            "x": float(px),
            "y": float(py),
            "position_mm": [float(cam_pt[0]), float(cam_pt[1]), float(cam_pt[2])],
        })

    return {
        "num_detected": len(corners),
        "corners": corners,
        "board_pose_mm": {
            "translation": [float(t[0]), float(t[1]), float(t[2])],
            "rvec": [float(r[0]), float(r[1]), float(r[2])],
        },
    }


def main() -> None:
    if len(sys.argv) != 8:
        sys.stderr.write(
            "usage: detect.py <image_path> <dictionary> <squares_x> "
            "<squares_y> <square_length_mm> <marker_length_mm> <intrinsics_json>\n"
        )
        sys.exit(2)
    result = detect(
        Path(sys.argv[1]).read_bytes(),
        sys.argv[2],
        int(sys.argv[3]),
        int(sys.argv[4]),
        float(sys.argv[5]),
        float(sys.argv[6]),
        json.loads(sys.argv[7]),
    )
    print(json.dumps(result))


if __name__ == "__main__":
    main()
