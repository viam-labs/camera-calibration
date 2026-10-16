#!/usr/bin/env python3
"""Auto-detect ChArUco board parameters from an image + depth frame."""

import json
import sys
from pathlib import Path

import cv2
import numpy as np
from cv2 import aruco


_DICTIONARIES = [
    ("DICT_4X4_50", aruco.DICT_4X4_50, 50),
    ("DICT_4X4_100", aruco.DICT_4X4_100, 100),
    ("DICT_4X4_250", aruco.DICT_4X4_250, 250),
    ("DICT_4X4_1000", aruco.DICT_4X4_1000, 1000),
    ("DICT_5X5_50", aruco.DICT_5X5_50, 50),
    ("DICT_5X5_100", aruco.DICT_5X5_100, 100),
    ("DICT_5X5_250", aruco.DICT_5X5_250, 250),
    ("DICT_5X5_1000", aruco.DICT_5X5_1000, 1000),
    ("DICT_6X6_50", aruco.DICT_6X6_50, 50),
    ("DICT_6X6_100", aruco.DICT_6X6_100, 100),
    ("DICT_6X6_250", aruco.DICT_6X6_250, 250),
    ("DICT_6X6_1000", aruco.DICT_6X6_1000, 1000),
    ("DICT_7X7_50", aruco.DICT_7X7_50, 50),
    ("DICT_7X7_100", aruco.DICT_7X7_100, 100),
    ("DICT_7X7_250", aruco.DICT_7X7_250, 250),
    ("DICT_7X7_1000", aruco.DICT_7X7_1000, 1000),
]

_MARKER_TO_SQUARE_RATIO = 0.75


def _detect_dictionary(image):
    best = None  # (name, dict_id, dict_size, ids array)
    for name, dict_id, dict_size in _DICTIONARIES:
        detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(dict_id), aruco.DetectorParameters())
        _, ids, _ = detector.detectMarkers(image)
        if ids is None or len(ids) == 0:
            continue
        if int(ids.max()) >= dict_size:
            continue
        count = len(ids)
        if best is None or count > len(best[3]) or (count == len(best[3]) and dict_size < best[2]):
            best = (name, dict_id, dict_size, ids.flatten())
    return best


def _candidate_grid_dims(num_markers):
    target = num_markers * 2
    for product in (target, target - 1):
        for w in range(3, int(np.sqrt(product)) + 1):
            if product % w == 0:
                h = product // w
                if h >= 3:
                    yield (w, h)
                    yield (h, w)


def _detect_grid(image, dict_id, num_markers, camera_matrix, dist):
    aruco_dict = aruco.getPredefinedDictionary(dict_id)
    best = None  # (w, h, corner_count, tvec, obj_points, img_points)
    for w, h in _candidate_grid_dims(num_markers):
        board = aruco.CharucoBoard((w, h), 1.0, _MARKER_TO_SQUARE_RATIO, aruco_dict)
        detector = aruco.CharucoDetector(board)
        corners, ids, _, _ = detector.detectBoard(image)
        if corners is None or ids is None or len(ids) < 6:
            continue
        obj_points, img_points = board.matchImagePoints(corners, ids)
        if obj_points is None or len(obj_points) < 6:
            continue
        ok, _, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist)
        if not ok:
            continue
        corner_count = len(ids)
        if best is None or corner_count > best[2]:
            best = (w, h, corner_count, tvec, obj_points, img_points)
    return best


def _scale_from_depth(depth, color_shape, obj_points, img_points, tvec):
    center_obj = np.mean(obj_points.reshape(-1, 3), axis=0)
    center_img = np.mean(img_points.reshape(-1, 2), axis=0)
    # Depth and color may have different resolutions; scale center coords to depth frame.
    scale_u = depth.shape[1] / color_shape[1]
    scale_v = depth.shape[0] / color_shape[0]
    u = int(round(center_img[0] * scale_u))
    v = int(round(center_img[1] * scale_v))
    # Sample a patch and take median of valid pixels — single center pixel is often 0 on
    # marker edges, reflective ink, or depth-shadow regions.
    half = 10
    u0, u1 = max(0, u - half), min(depth.shape[1], u + half + 1)
    v0, v1 = max(0, v - half), min(depth.shape[0], v + half + 1)
    patch = depth[v0:v1, u0:u1]
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    depth_mm = float(np.median(valid))
    distance_in_units = float(tvec.flatten()[2]) + float(center_obj[2])
    if distance_in_units <= 0:
        return None
    return depth_mm / distance_in_units


def discover(rgb_bytes, depth_bytes, intrinsics):
    rgb_arr = np.frombuffer(rgb_bytes, dtype=np.uint8)
    gray = cv2.imdecode(rgb_arr, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise ValueError("failed to decode rgb image")

    depth_arr = np.frombuffer(depth_bytes, dtype=np.uint8)
    depth = cv2.imdecode(depth_arr, cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise ValueError("failed to decode depth image")
    if depth.ndim == 3:
        depth = depth[:, :, 0]

    dict_result = _detect_dictionary(gray)
    if dict_result is None:
        raise ValueError("no ArUco markers detected. Check that the camera can see the entire board, or add the charuco pose_tracker manually (see README).")
    dict_name, dict_id, _, ids = dict_result

    camera_matrix = np.array([
        [intrinsics["fx"], 0.0, intrinsics["cx"]],
        [0.0, intrinsics["fy"], intrinsics["cy"]],
        [0.0, 0.0, 1.0],
    ])
    dist = np.array(intrinsics.get("distortion") or [0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    grid = _detect_grid(gray, dict_id, len(ids), camera_matrix, dist)
    if grid is None:
        raise ValueError(f"detected dictionary {dict_name} but could not fit ChArUco grid")
    w, h, _, tvec, obj_points, img_points = grid

    scale = _scale_from_depth(depth, gray.shape, obj_points, img_points, tvec)
    if scale is None:
        raise ValueError("could not read valid depth at the board center. Check the camera is within the depth sensor's working range and the board isn't highly reflective.")

    return {
        "dictionary": dict_name,
        "squares_x": w,
        "squares_y": h,
        "square_length_mm": round(scale, 2),
        "marker_length_mm": round(scale * _MARKER_TO_SQUARE_RATIO, 2),
    }


def main():
    if len(sys.argv) != 4:
        sys.stderr.write("usage: discover.py <rgb_path> <depth_path> <intrinsics_json>\n")
        sys.exit(2)
    try:
        result = discover(
            Path(sys.argv[1]).read_bytes(),
            Path(sys.argv[2]).read_bytes(),
            json.loads(sys.argv[3]),
        )
    except ValueError as e:
        sys.stderr.write(str(e) + "\n")
        sys.exit(1)
    print(json.dumps(result))


if __name__ == "__main__":
    main()
