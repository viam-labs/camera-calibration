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


def _synthetic_board_image() -> bytes:
    """Generate a PNG-encoded image of the ChArUco board (idealized, no distortion)."""
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


def test_detects_all_corners():
    """On a clean synthetic board, all interior corners should be detected."""
    image_bytes = _synthetic_board_image()
    result = detect(
        image_bytes,
        _DICT,
        _SQUARES_X,
        _SQUARES_Y,
        _SQUARE_LENGTH_MM,
        _MARKER_LENGTH_MM,
    )
    # For an X-by-Y ChArUco board, there are (X-1) * (Y-1) interior corners.
    expected = (_SQUARES_X - 1) * (_SQUARES_Y - 1)
    assert result["num_detected"] == expected


def test_corner_positions_inside_image():
    """Detected corner positions should be within the image bounds."""
    image_bytes = _synthetic_board_image()
    result = detect(
        image_bytes,
        _DICT,
        _SQUARES_X,
        _SQUARES_Y,
        _SQUARE_LENGTH_MM,
        _MARKER_LENGTH_MM,
    )
    for corner in result["corners"]:
        assert 0 <= corner["x"] < _IMAGE_SIZE_PX[0]
        assert 0 <= corner["y"] < _IMAGE_SIZE_PX[1]


def test_unknown_dictionary_raises():
    with pytest.raises(ValueError, match="unknown dictionary"):
        detect(b"", "DICT_BOGUS", 5, 5, 40.0, 30.0)


def test_unreadable_image_raises():
    with pytest.raises(ValueError, match="failed to decode"):
        detect(b"not an image", _DICT, 5, 5, 40.0, 30.0)
