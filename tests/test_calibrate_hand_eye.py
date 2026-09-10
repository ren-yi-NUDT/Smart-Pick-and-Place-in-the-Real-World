import cv2
import numpy as np

from tools.calibrate_hand_eye import (
    _pose_delta,
    evaluate_extrinsic,
    solve_hand_eye,
)


def _pose(rx, ry, rz, x, y, z):
    sx, cx = np.sin(rx), np.cos(rx)
    sy, cy = np.sin(ry), np.cos(ry)
    sz, cz = np.sin(rz), np.cos(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = [x, y, z]
    return T


def test_hand_eye_solver_returns_gripper_to_camera_direction():
    T_gripper_camera = _pose(0.08, -0.04, 1.62, 0.071, 0.023, 0.095)
    T_base_target = _pose(0.1, -0.05, 0.2, 0.45, -0.2, 0.12)
    samples = []
    for i in range(20):
        T_base_gripper = _pose(
            0.22 * np.sin(i * 0.7),
            0.28 * np.cos(i * 0.5),
            0.18 * np.sin(i * 0.3),
            0.10 + 0.025 * (i % 5),
            -0.22 + 0.018 * (i % 4),
            0.42 + 0.012 * (i % 3),
        )
        T_camera_target = (
            np.linalg.inv(T_gripper_camera)
            @ np.linalg.inv(T_base_gripper)
            @ T_base_target
        )
        samples.append({
            "T_base_gripper": T_base_gripper,
            "T_camera_target": T_camera_target,
        })

    results, diversity = solve_hand_eye(samples)
    best = results[0]
    translation_m, rotation_deg = _pose_delta(
        best["matrix"], T_gripper_camera
    )

    assert diversity["rotation_axis_rank"] == 3
    assert translation_m < 1e-5
    assert rotation_deg < 1e-3
    assert evaluate_extrinsic(best["matrix"], samples)["quality_score"] < 1e-4
