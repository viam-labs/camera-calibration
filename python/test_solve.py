"""Tests for solve.solve."""

import cv2
import numpy as np
import pytest

from solve import solve


def _se3_inverse(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    inv = np.eye(4)
    inv[:3, :3] = R.T
    inv[:3, 3] = -R.T @ t
    return inv


def _make_transform(rvec: list, translation: list) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation
    return T


def _to_transform_dict(T: np.ndarray) -> dict:
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return {
        "translation": T[:3, 3].tolist(),
        "rvec": rvec.flatten().tolist(),
    }


def _synthetic_stations(x_true: np.ndarray, n_stations: int, seed: int = 42) -> list:
    """Generate n_stations with a known X_true and a fixed board-in-base pose."""
    rng = np.random.default_rng(seed)
    T_bw = np.eye(4)
    T_bw[:3, 3] = [200.0, 100.0, 0.0]

    stations = []
    for _ in range(n_stations):
        rvec_arm = rng.uniform(-0.3, 0.3, 3)
        R_arm, _ = cv2.Rodrigues(rvec_arm)
        T_be = np.eye(4)
        T_be[:3, :3] = R_arm
        T_be[:3, 3] = rng.uniform(-100.0, 100.0, 3)

        # T_bw = T_be @ X @ T_cw  =>  T_cw = inv(X) @ inv(T_be) @ T_bw
        T_cw = _se3_inverse(x_true) @ _se3_inverse(T_be) @ T_bw

        stations.append({
            "T_be": _to_transform_dict(T_be),
            "T_cw": _to_transform_dict(T_cw),
        })
    return stations


def test_round_trip_tsai():
    """Recovered X should match the synthesized X within tight tolerance."""
    x_true = _make_transform([0.1, -0.2, 0.05], [50.0, 30.0, 10.0])
    stations = _synthetic_stations(x_true, n_stations=15)

    result = solve(stations, method="tsai")

    rec = result["camera_in_gripper_mm"]
    R_rec, _ = cv2.Rodrigues(np.array(rec["rvec"]))
    t_rec = np.array(rec["translation"])

    R_true = x_true[:3, :3]
    t_true = x_true[:3, 3]

    R_err = R_rec.T @ R_true
    rvec_err, _ = cv2.Rodrigues(R_err)
    angle_err_deg = float(np.degrees(np.linalg.norm(rvec_err)))
    t_err = float(np.linalg.norm(t_rec - t_true))

    assert t_err < 1.0, f"translation error {t_err:.4f}mm exceeds 1mm tolerance"
    assert angle_err_deg < 0.1, f"rotation error {angle_err_deg:.4f}° exceeds 0.1° tolerance"


def test_insufficient_stations_raises():
    x_true = _make_transform([0.1, -0.2, 0.05], [50.0, 30.0, 10.0])
    stations = _synthetic_stations(x_true, n_stations=2)
    with pytest.raises(ValueError, match="at least 3 stations"):
        solve(stations, method="tsai")


def test_unknown_method_raises():
    x_true = _make_transform([0.1, -0.2, 0.05], [50.0, 30.0, 10.0])
    stations = _synthetic_stations(x_true, n_stations=5)
    with pytest.raises(ValueError, match="unknown method"):
        solve(stations, method="bogus_method")
