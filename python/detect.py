#!/usr/bin/env python3
"""ChArUco corner detection. Runnable as a CLI for pyrunner."""

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
) -> dict:
    """Detect ChArUco corners in an image.

    Args:
        image_bytes: raw image bytes (JPEG, PNG, etc.). Decoded via cv2.imdecode.
        dictionary: ArUco dictionary name (e.g., "DICT_5X5_100").
        squares_x, squares_y: board grid dimensions (number of squares).
        square_length_mm: physical chessboard square size in mm.
        marker_length_mm: physical ArUco marker size in mm; must be < square_length_mm.

    Returns:
        {
            "num_detected": int,
            "corners": [{"id": int, "x": float, "y": float}, ...],
        }
        `corners` is empty if no corners are detected. IDs match the board's
        interior-corner numbering (0 to (squares_x-1)*(squares_y-1) - 1).

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

    if charuco_corners is None or charuco_ids is None:
        return {"num_detected": 0, "corners": []}

    corners = []
    for i in range(len(charuco_ids)):
        cid = int(charuco_ids[i].item())
        cx, cy = charuco_corners[i][0]
        corners.append({"id": cid, "x": float(cx), "y": float(cy)})

    return {"num_detected": len(corners), "corners": corners}


def main() -> None:
    if len(sys.argv) != 7:
        sys.stderr.write(
            "usage: detect.py <image_path> <dictionary> <squares_x> "
            "<squares_y> <square_length_mm> <marker_length_mm>\n"
        )
        sys.exit(2)
    result = detect(
        Path(sys.argv[1]).read_bytes(),
        sys.argv[2],
        int(sys.argv[3]),
        int(sys.argv[4]),
        float(sys.argv[5]),
        float(sys.argv[6]),
    )
    print(json.dumps(result))


if __name__ == "__main__":
    main()
