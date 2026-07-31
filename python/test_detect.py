"""Tests for detect.detect."""

import cv2
import pytest
from cv2 import aruco

from detect import detect

_DICT = "DICT_5X5_100"
_SQUARES_X = 7
_SQUARES_Y = 5
_SQUARE_LENGTH_MM = 40.0
_MARKER_LENGTH_MM = 30.0
_IMAGE_SIZE_PX = (1400, 1000)  # (width, height)

# Placeholder intrinsics for a hypothetical camera whose image happens to match
# _IMAGE_SIZE_PX. Not physically accurate; sufficient for PnP to converge on
# the synthetic (idealized) board image so we can validate the wiring.
_INTRINSICS = {
    "fx": 1000.0,
    "fy": 1000.0,
    "cx": _IMAGE_SIZE_PX[0] / 2,
    "cy": _IMAGE_SIZE_PX[1] / 2,
    "distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
}


def _synthetic_board_image() -> bytes:
    aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, _DICT))
    board = aruco.CharucoBoard(
        (_SQUARES_X, _SQUARES_Y),
        _SQUARE_LENGTH_MM,
        _MARKER_LENGTH_MM,
        aruco_dict,
    )
    img = board.generateImage(_IMAGE_SIZE_PX)
    ok, buf = cv2.imencode(".png", img)
    assert ok, "failed to encode synthetic board image"
    return buf.tobytes()


def _run(image_bytes: bytes = None) -> dict:
    if image_bytes is None:
        image_bytes = _synthetic_board_image()
    return detect(
        image_bytes,
        _DICT,
        _SQUARES_X,
        _SQUARES_Y,
        _SQUARE_LENGTH_MM,
        _MARKER_LENGTH_MM,
        _INTRINSICS,
    )


def test_detects_all_corners():
    """On a clean synthetic board, all interior corners should be detected."""
    result = _run()
    expected = (_SQUARES_X - 1) * (_SQUARES_Y - 1)
    assert result["num_detected"] == expected


def test_corner_positions_inside_image():
    """Detected corner pixel positions should be within the image bounds."""
    result = _run()
    for corner in result["corners"]:
        assert 0 <= corner["x"] < _IMAGE_SIZE_PX[0]
        assert 0 <= corner["y"] < _IMAGE_SIZE_PX[1]


def test_pnp_returns_finite_3d_positions():
    """Each detected corner should have a finite 3D position, and the board pose should be populated."""
    result = _run()
    assert result["board_pose_mm"] is not None
    assert len(result["board_pose_mm"]["translation"]) == 3
    assert len(result["board_pose_mm"]["rvec"]) == 3
    for corner in result["corners"]:
        assert len(corner["position_mm"]) == 3
        for v in corner["position_mm"]:
            assert v == v  # not NaN
            assert abs(v) < 1e6  # finite-ish sanity


def test_unknown_dictionary_raises():
    with pytest.raises(ValueError, match="unknown dictionary"):
        detect(b"", "DICT_BOGUS", 5, 5, 40.0, 30.0, _INTRINSICS)


def test_unreadable_image_raises():
    with pytest.raises(ValueError, match="failed to decode"):
        detect(b"not an image", _DICT, 5, 5, 40.0, 30.0, _INTRINSICS)
