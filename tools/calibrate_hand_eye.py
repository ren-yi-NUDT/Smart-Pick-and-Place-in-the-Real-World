#!/home/zz/anaconda3/envs/anygrasp/bin/python3
# -*- coding: utf-8 -*-
"""逐臂 D455 眼在手上标定（手动移动，脚本只采集、不控制机械臂）。

标定板必须在一次采集期间保持固定。把机械臂移稳后，在预览窗口按 ``c``
采集；``u`` 撤销上一组，``q`` 提前求解（标定至少 15 组），Esc 放弃。

用法：
    python tools/calibrate_hand_eye.py calibrate --side left
    python tools/calibrate_hand_eye.py verify --side left
    python tools/calibrate_hand_eye.py calibrate --side right
    python tools/calibrate_hand_eye.py verify --side right

标定结果是 ``T_gripper_camera``，即项目配置所需的
``Link7 -> cam_link_grasp``（右臂对应 R_ 前缀），保存前会备份配置。
"""

import argparse
import json
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from termcolor import cprint


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_ROOT / "robot_config.json"
SESSION_ROOT = PROJECT_ROOT / "log" / "hand_eye"
BACKUP_ROOT = PROJECT_ROOT / "log" / "calibration_backups"

CAMERA_W, CAMERA_H, CAMERA_FPS = 640, 480, 30
FRAMES_PER_SAMPLE = 12
MIN_VALID_FRAMES = 8
MIN_CHARUCO_CORNERS = 10
MAX_REPROJECTION_ERROR_PX = 1.5
MIN_CALIBRATION_SAMPLES = 15

# 同一次采样期间机械臂必须静止；相邻有效位姿也不能几乎重复。
MAX_CAPTURE_MOTION_M = 0.0015
MAX_CAPTURE_MOTION_DEG = 0.3
MIN_NEW_TRANSLATION_M = 0.015
MIN_NEW_ROTATION_DEG = 5.0

# 用“固定标定板在 base 中的重建一致性”评估外参。
QUALITY_LIMITS = {
    "mean_translation_mm": 5.0,
    "max_translation_mm": 10.0,
    "mean_rotation_deg": 1.5,
    "max_rotation_deg": 3.0,
}

HAND_EYE_METHODS = {
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def _rotation_error_deg(R_a, R_b):
    value = (np.trace(np.asarray(R_a).T @ np.asarray(R_b)) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(value, -1.0, 1.0))))


def _mean_transform(transforms):
    """Chordal rotation mean plus median translation."""
    transforms = [np.asarray(T, dtype=float) for T in transforms]
    M = np.mean([T[:3, :3] for T in transforms], axis=0)
    U, _, Vt = np.linalg.svd(M)
    correction = np.diag([1.0, 1.0, np.linalg.det(U @ Vt)])

    result = np.eye(4)
    result[:3, :3] = U @ correction @ Vt
    result[:3, 3] = np.median([T[:3, 3] for T in transforms], axis=0)
    return result


def _pose_delta(T_a, T_b):
    translation_m = float(
        np.linalg.norm(np.asarray(T_a)[:3, 3] - np.asarray(T_b)[:3, 3])
    )
    rotation_deg = _rotation_error_deg(
        np.asarray(T_a)[:3, :3], np.asarray(T_b)[:3, :3]
    )
    return translation_m, rotation_deg


def _sample_matrices(sample):
    return (
        np.asarray(sample["T_base_gripper"], dtype=float),
        np.asarray(sample["T_camera_target"], dtype=float),
    )


def evaluate_extrinsic(T_gripper_camera, samples):
    """重建固定板的 base 位姿，并统计各组的一致性。"""
    board_poses = []
    for sample in samples:
        T_base_gripper, T_camera_target = _sample_matrices(sample)
        board_poses.append(
            T_base_gripper @ T_gripper_camera @ T_camera_target
        )

    center = _mean_transform(board_poses)
    translation_errors = [
        np.linalg.norm(T[:3, 3] - center[:3, 3]) * 1000.0
        for T in board_poses
    ]
    rotation_errors = [
        _rotation_error_deg(center[:3, :3], T[:3, :3]) for T in board_poses
    ]
    metrics = {
        "mean_translation_mm": float(np.mean(translation_errors)),
        "max_translation_mm": float(np.max(translation_errors)),
        "mean_rotation_deg": float(np.mean(rotation_errors)),
        "max_rotation_deg": float(np.max(rotation_errors)),
    }
    metrics["quality_score"] = max(
        metrics[key] / limit for key, limit in QUALITY_LIMITS.items()
    )
    return metrics


def _validate_transform(T):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError("外参不是有限的 4x4 矩阵")
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError("外参齐次矩阵最后一行无效")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-5):
        raise ValueError("外参旋转矩阵不正交")
    if np.linalg.det(R) < 0.999:
        raise ValueError("外参旋转矩阵行列式无效")
    if np.linalg.norm(T[:3, 3]) > 0.5:
        raise ValueError("相机距 Link7 超过 0.5m，疑似坐标方向错误")
    return T


def _check_motion_diversity(samples):
    rotations = [_sample_matrices(sample)[0][:3, :3] for sample in samples]
    reference = rotations[0]
    vectors = []
    for rotation in rotations[1:]:
        rvec, _ = cv2.Rodrigues(reference.T @ rotation)
        vectors.append(rvec.reshape(3))
    vectors = np.asarray(vectors)
    span_deg = float(np.degrees(np.max(np.linalg.norm(vectors, axis=1))))
    axis_rank = int(np.linalg.matrix_rank(vectors, tol=np.deg2rad(3.0)))
    if span_deg < 20.0 or axis_rank < 2:
        raise ValueError(
            f"机械臂旋转变化不足（最大 {span_deg:.1f}°, 轴秩 {axis_rank}）；"
            "请增加绕不同轴旋转的位姿"
        )
    return span_deg, axis_rank


def solve_hand_eye(samples):
    """运行多种 OpenCV 解法并按固定板重建一致性排序。"""
    if len(samples) < MIN_CALIBRATION_SAMPLES:
        raise ValueError(
            f"至少需要 {MIN_CALIBRATION_SAMPLES} 个有效位姿，当前 {len(samples)}"
        )
    span_deg, axis_rank = _check_motion_diversity(samples)

    matrices = [_sample_matrices(sample) for sample in samples]
    R_gripper2base = [T_bg[:3, :3] for T_bg, _ in matrices]
    t_gripper2base = [T_bg[:3, 3].reshape(3, 1) for T_bg, _ in matrices]
    R_target2cam = [T_ct[:3, :3] for _, T_ct in matrices]
    t_target2cam = [T_ct[:3, 3].reshape(3, 1) for _, T_ct in matrices]

    results = []
    for name, method in HAND_EYE_METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(
                R_gripper2base,
                t_gripper2base,
                R_target2cam,
                t_target2cam,
                method=method,
            )
            T_gripper_camera = np.eye(4)
            T_gripper_camera[:3, :3] = R
            T_gripper_camera[:3, 3] = np.asarray(t).reshape(3)
            _validate_transform(T_gripper_camera)
            metrics = evaluate_extrinsic(T_gripper_camera, samples)
            results.append({
                "method": name,
                "matrix": T_gripper_camera,
                "metrics": metrics,
            })
        except (cv2.error, TypeError, ValueError, np.linalg.LinAlgError) as exc:
            cprint(f"[hand-eye] {name} 求解失败: {exc}", "yellow")

    if not results:
        raise RuntimeError("所有手眼标定方法都失败，请重新采集更多旋转位姿")
    results.sort(key=lambda item: item["metrics"]["quality_score"])
    return results, {"rotation_span_deg": span_deg, "rotation_axis_rank": axis_rank}


def _atomic_json_write(path, data):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    try:
        with tmp.open("w", encoding="utf-8") as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
        os.replace(str(tmp), str(path))
    finally:
        if tmp.exists():
            tmp.unlink()


def _save_extrinsic(side, matrix, method, metrics):
    matrix = _validate_transform(matrix)
    with CONFIG_PATH.open("r", encoding="utf-8") as stream:
        config = json.load(stream)

    arm = config["arms"][side]
    extrinsic = arm["camera_extrinsic"]
    expected_parent = arm["arm_end_link_name"]
    if extrinsic.get("parent_frame") != expected_parent:
        raise ValueError(
            f"配置 parent_frame={extrinsic.get('parent_frame')}，"
            f"但机械臂末端是 {expected_parent}"
        )
    if not extrinsic.get("child_frame"):
        raise ValueError(f"{side} camera_extrinsic 缺少 child_frame")

    BACKUP_ROOT.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = BACKUP_ROOT / f"robot_config_before_hand_eye_{side}_{timestamp}.json"
    shutil.copy2(str(CONFIG_PATH), str(backup))

    extrinsic["matrix"] = matrix.tolist()
    _atomic_json_write(CONFIG_PATH, config)
    cprint(f"[hand-eye] 已更新 {side} camera_extrinsic", "green")
    cprint(f"[hand-eye] 旧配置备份: {backup.relative_to(PROJECT_ROOT)}", "green")
    cprint(f"[hand-eye] 方法: {method}, score={metrics['quality_score']:.3f}", "green")
    cprint("[hand-eye] 请重启 ROS Bringup 后再运行视觉任务", "yellow")


def _load_config(side):
    with CONFIG_PATH.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    if side not in config.get("arms", {}):
        raise ValueError(f"robot_config.json 中没有 {side} 臂")
    arm = config["arms"][side]
    for key in ("camera_serial", "base_link_name", "arm_end_link_name"):
        if not arm.get(key):
            raise ValueError(f"{side} 臂缺少配置: {key}")
    return arm


def _load_charuco_helpers():
    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))
    from tools.calibrate_arms import (  # pylint: disable=import-outside-toplevel
        CHARUCO_DICT,
        CHARUCO_MARKER_LENGTH,
        CHARUCO_SQUARE_LENGTH,
        CHARUCO_SQUARES_X,
        CHARUCO_SQUARES_Y,
        create_charuco_board,
    )
    return {
        "board": create_charuco_board(),
        "dictionary": CHARUCO_DICT,
        "squares_x": CHARUCO_SQUARES_X,
        "squares_y": CHARUCO_SQUARES_Y,
        "square_length_m": CHARUCO_SQUARE_LENGTH,
        "marker_length_m": CHARUCO_MARKER_LENGTH,
    }


def _open_camera(serial):
    import pyrealsense2 as rs  # pylint: disable=import-outside-toplevel

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    # Match core/camera.py's runtime RGB-D profile; hand-eye uses the color
    # frame, but profile selection must stay identical to visual grasping.
    config.enable_stream(
        rs.stream.depth, CAMERA_W, CAMERA_H, rs.format.z16, CAMERA_FPS
    )
    config.enable_stream(
        rs.stream.color, CAMERA_W, CAMERA_H, rs.format.bgr8, CAMERA_FPS
    )
    try:
        profile = pipeline.start(config)
        intr = profile.get_stream(
            rs.stream.color
        ).as_video_stream_profile().get_intrinsics()
        K = np.array([
            [intr.fx, 0.0, intr.ppx],
            [0.0, intr.fy, intr.ppy],
            [0.0, 0.0, 1.0],
        ])
        dist = np.asarray(intr.coeffs, dtype=float)
        for _ in range(30):
            pipeline.wait_for_frames()
        return pipeline, K, dist
    except Exception:
        try:
            pipeline.stop()
        except Exception:
            pass
        raise


def _check_configured_intrinsics(arm, K):
    configured = arm.get("camera_intrinsics", {})
    expected = np.array([
        configured.get("fx"), configured.get("fy"),
        configured.get("cx"), configured.get("cy"),
    ], dtype=float)
    actual = np.array([K[0, 0], K[1, 1], K[0, 2], K[1, 2]])
    if not np.all(np.isfinite(expected)) or np.max(np.abs(expected - actual)) > 2.0:
        raise ValueError(
            "D455 当前内参与 robot_config.json 相差超过 2px；"
            "请先同步相机内参，避免外参吸收内参误差"
        )


def _color_frame(pipeline):
    frames = pipeline.wait_for_frames()
    frame = frames.get_color_frame()
    if not frame:
        raise RuntimeError("D455 未返回彩色帧")
    return np.asanyarray(frame.get_data())


def _init_tf(base_frame, gripper_frame):
    import rospy  # pylint: disable=import-outside-toplevel
    import tf  # pylint: disable=import-outside-toplevel

    try:
        rospy.init_node("calibrate_hand_eye", anonymous=True, disable_signals=True)
    except rospy.exceptions.ROSException:
        pass
    listener = tf.TransformListener()
    listener.waitForTransform(
        base_frame, gripper_frame, rospy.Time(0), rospy.Duration(8.0)
    )
    return listener, rospy, tf


def _lookup_tf(listener, rospy, tf_module, base_frame, gripper_frame):
    trans, quat = listener.lookupTransform(
        base_frame, gripper_frame, rospy.Time(0)
    )
    T = np.eye(4)
    T[:3, :3] = tf_module.transformations.quaternion_matrix(quat)[:3, :3]
    T[:3, 3] = trans
    return T


def _detect_charuco(detector, board, image, K, dist):
    corners, ids, marker_corners, marker_ids = detector.detectBoard(image)
    if ids is None or len(ids) < MIN_CHARUCO_CORNERS:
        return None
    obj_points, img_points = board.matchImagePoints(corners, ids)
    if obj_points is None or len(obj_points) < MIN_CHARUCO_CORNERS:
        return None
    ok, rvec, tvec = cv2.solvePnP(
        obj_points, img_points, K, dist, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok:
        return None
    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, K, dist)
    residual = projected.reshape(-1, 2) - img_points.reshape(-1, 2)
    reprojection_px = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))

    T_camera_target = np.eye(4)
    T_camera_target[:3, :3] = cv2.Rodrigues(rvec)[0]
    T_camera_target[:3, 3] = tvec.reshape(3)
    return {
        "matrix": T_camera_target,
        "corners": int(len(ids)),
        "reprojection_px": reprojection_px,
        "charuco_corners": corners,
        "charuco_ids": ids,
        "marker_corners": marker_corners,
        "marker_ids": marker_ids,
        "rvec": rvec,
        "tvec": tvec,
    }


def _estimate_board_pose(detector, board, images, K, dist):
    detections = []
    for image in images:
        detection = _detect_charuco(detector, board, image, K, dist)
        if detection and detection["reprojection_px"] <= MAX_REPROJECTION_ERROR_PX:
            detection["image"] = image
            detections.append(detection)
    if len(detections) < MIN_VALID_FRAMES:
        return None

    center = _mean_transform([item["matrix"] for item in detections])
    translation_errors = np.array([
        np.linalg.norm(item["matrix"][:3, 3] - center[:3, 3])
        for item in detections
    ])
    rotation_errors = np.array([
        _rotation_error_deg(center[:3, :3], item["matrix"][:3, :3])
        for item in detections
    ])
    keep = (
        (translation_errors <= max(0.003, 3.0 * np.median(translation_errors)))
        & (rotation_errors <= max(0.5, 3.0 * np.median(rotation_errors)))
    )
    inliers = [item for item, accepted in zip(detections, keep) if accepted]
    if len(inliers) < MIN_VALID_FRAMES:
        return None

    best = min(inliers, key=lambda item: item["reprojection_px"])
    return {
        "matrix": _mean_transform([item["matrix"] for item in inliers]),
        "valid_frames": len(inliers),
        "corners": int(np.median([item["corners"] for item in inliers])),
        "reprojection_px": float(np.mean([
            item["reprojection_px"] for item in inliers
        ])),
        "best": best,
    }


def _annotated_image(detection):
    image = detection["image"].copy()
    if detection["marker_ids"] is not None:
        cv2.aruco.drawDetectedMarkers(
            image, detection["marker_corners"], detection["marker_ids"]
        )
    cv2.aruco.drawDetectedCornersCharuco(
        image, detection["charuco_corners"], detection["charuco_ids"]
    )
    return image


def _is_duplicate(T_base_gripper, samples):
    for sample in samples:
        previous, _ = _sample_matrices(sample)
        translation_m, rotation_deg = _pose_delta(previous, T_base_gripper)
        if (
            translation_m < MIN_NEW_TRANSLATION_M
            and rotation_deg < MIN_NEW_ROTATION_DEG
        ):
            return True
    return False


def _write_session(session_dir, metadata, samples):
    data = dict(metadata)
    data["samples"] = samples
    _atomic_json_write(session_dir / "samples.json", data)


def _collect(side, target_count, minimum_count, mode):
    arm = _load_config(side)
    charuco = _load_charuco_helpers()
    board = charuco.pop("board")
    detector = cv2.aruco.CharucoDetector(board)
    base_frame = arm["base_link_name"]
    gripper_frame = arm["arm_end_link_name"]

    listener, rospy, tf_module = _init_tf(base_frame, gripper_frame)
    pipeline = None
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    session_dir = SESSION_ROOT / f"{side}_{mode}_{timestamp}"
    session_dir.mkdir(parents=True, exist_ok=False)
    samples = []

    try:
        pipeline, K, dist = _open_camera(arm["camera_serial"])
        _check_configured_intrinsics(arm, K)
        metadata = {
            "version": 1,
            "mode": mode,
            "side": side,
            "timestamp": datetime.now().isoformat(),
            "camera_serial": arm["camera_serial"],
            "base_frame": base_frame,
            "gripper_frame": gripper_frame,
            "intrinsics": {"K": K.tolist(), "distortion": dist.tolist()},
            "board": charuco,
        }
        _write_session(session_dir, metadata, samples)

        cprint(f"\n[hand-eye] {side} 臂 {mode}，数据目录: {session_dir}", "cyan")
        cprint("[hand-eye] 标定板保持固定；手动移稳机械臂后，在画面窗口按 c", "yellow")
        cprint("[hand-eye] c=采集  u=撤销  q=完成  Esc=退出", "yellow")

        while True:
            image = _color_frame(pipeline)
            preview = image.copy()
            detection = _detect_charuco(detector, board, image, K, dist)
            ready = bool(
                detection
                and detection["reprojection_px"] <= MAX_REPROJECTION_ERROR_PX
            )
            status = "BOARD READY" if ready else "BOARD NOT READY"
            color = (0, 220, 0) if ready else (0, 0, 255)
            if detection:
                preview = _annotated_image(dict(detection, image=image))
                status += (
                    f" corners={detection['corners']} "
                    f"err={detection['reprojection_px']:.2f}px"
                )
            cv2.putText(preview, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, color, 2)
            cv2.putText(preview, f"samples {len(samples)}/{target_count}",
                        (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 255, 255), 2)
            cv2.imshow(f"Hand-eye {side}: c capture / u undo / q solve", preview)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                cprint("[hand-eye] 已退出，未求解；已采数据仍保留", "yellow")
                return None, session_dir
            if key == ord("u"):
                if samples:
                    samples.pop()
                    _write_session(session_dir, metadata, samples)
                    cprint(f"[hand-eye] 已撤销，剩余 {len(samples)} 组", "yellow")
                continue
            if key == ord("q"):
                if len(samples) < minimum_count:
                    cprint(
                        f"[hand-eye] 至少需要 {minimum_count} 组，当前 {len(samples)}",
                        "red",
                    )
                    continue
                break
            if key != ord("c"):
                continue

            T_before = _lookup_tf(
                listener, rospy, tf_module, base_frame, gripper_frame
            )
            images = [_color_frame(pipeline) for _ in range(FRAMES_PER_SAMPLE)]
            T_after = _lookup_tf(
                listener, rospy, tf_module, base_frame, gripper_frame
            )
            moved_m, moved_deg = _pose_delta(T_before, T_after)
            if moved_m > MAX_CAPTURE_MOTION_M or moved_deg > MAX_CAPTURE_MOTION_DEG:
                cprint(
                    f"[hand-eye] 拒绝：采集时机械臂移动了 "
                    f"{moved_m*1000:.1f}mm/{moved_deg:.2f}°",
                    "red",
                )
                continue

            estimate = _estimate_board_pose(detector, board, images, K, dist)
            if estimate is None:
                cprint(
                    f"[hand-eye] 拒绝：需要至少 {MIN_VALID_FRAMES}/{FRAMES_PER_SAMPLE} "
                    f"帧稳定检测且重投影误差 ≤{MAX_REPROJECTION_ERROR_PX}px",
                    "red",
                )
                continue

            T_base_gripper = _mean_transform([T_before, T_after])
            if _is_duplicate(T_base_gripper, samples):
                cprint("[hand-eye] 拒绝：与已有机械臂位姿过于接近", "red")
                continue

            index = len(samples) + 1
            image_name = f"sample_{index:02d}.png"
            cv2.imwrite(
                str(session_dir / image_name),
                _annotated_image(estimate["best"]),
            )
            samples.append({
                "index": index,
                "timestamp": datetime.now().isoformat(),
                "T_base_gripper": T_base_gripper.tolist(),
                "T_camera_target": estimate["matrix"].tolist(),
                "valid_frames": estimate["valid_frames"],
                "charuco_corners": estimate["corners"],
                "reprojection_px": estimate["reprojection_px"],
                "image": image_name,
            })
            _write_session(session_dir, metadata, samples)
            cprint(
                f"[hand-eye] 接受第 {index} 组: "
                f"corners≈{estimate['corners']}, "
                f"error={estimate['reprojection_px']:.3f}px",
                "green",
            )
            if len(samples) >= target_count:
                break

        return samples, session_dir
    finally:
        cv2.destroyAllWindows()
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass


def _print_result(result):
    metrics = result["metrics"]
    cprint(
        f"  {result['method']:<10} score={metrics['quality_score']:.3f}  "
        f"平移 mean/max={metrics['mean_translation_mm']:.2f}/"
        f"{metrics['max_translation_mm']:.2f}mm  "
        f"旋转 mean/max={metrics['mean_rotation_deg']:.2f}/"
        f"{metrics['max_rotation_deg']:.2f}°",
        "green" if metrics["quality_score"] <= 1.0 else "yellow",
    )


def calibrate(side, samples_target):
    samples, session_dir = _collect(
        side, samples_target, MIN_CALIBRATION_SAMPLES, "calibrate"
    )
    if not samples:
        return False
    try:
        results, diversity = solve_hand_eye(samples)
    except (ValueError, RuntimeError) as exc:
        cprint(f"[hand-eye] 无法求解: {exc}", "red")
        return False

    cprint(
        f"\n[hand-eye] 旋转覆盖 {diversity['rotation_span_deg']:.1f}°，"
        f"轴秩 {diversity['rotation_axis_rank']}",
        "cyan",
    )
    cprint("[hand-eye] 各方法固定板重建误差：", "cyan")
    for result in results:
        _print_result(result)

    best = results[0]
    cprint(f"\n[hand-eye] 推荐 {best['method']}，T_gripper_camera:", "cyan")
    for row in best["matrix"]:
        print("  [" + " ".join(f"{value: .8f}" for value in row) + "]")

    result_path = session_dir / "result.json"
    _atomic_json_write(result_path, {
        "method": best["method"],
        "matrix": best["matrix"].tolist(),
        "metrics": best["metrics"],
        "diversity": diversity,
    })
    if best["metrics"]["quality_score"] > 1.0:
        cprint("[hand-eye] 质量未达标，不允许写入配置；请增加姿态多样性后重采", "red")
        return False

    token = f"SAVE-{side.upper()}"
    answer = input(f"输入 {token} 写入 robot_config.json，其余输入不保存: ").strip()
    if answer != token:
        cprint(f"[hand-eye] 未写入配置；结果保存在 {result_path}", "yellow")
        return True
    _save_extrinsic(side, best["matrix"], best["method"], best["metrics"])
    return True


def verify(side, samples_target):
    arm = _load_config(side)
    matrix = _validate_transform(
        np.asarray(arm["camera_extrinsic"]["matrix"], dtype=float)
    )
    samples, session_dir = _collect(side, samples_target, 3, "verify")
    if not samples:
        return False
    metrics = evaluate_extrinsic(matrix, samples)
    result = {"method": "CONFIG", "matrix": matrix, "metrics": metrics}
    cprint("\n[hand-eye] 独立验证结果：", "cyan")
    _print_result(result)
    _atomic_json_write(session_dir / "verify_result.json", metrics)
    if metrics["quality_score"] <= 1.0:
        cprint("[hand-eye] 验证通过，可用于视觉抓取", "green")
        return True
    cprint("[hand-eye] 验证未通过，请勿进行视觉抓取", "red")
    return False


def main():
    parser = argparse.ArgumentParser(
        description="逐臂 D455 眼在手上标定；仅采集，不自动移动机械臂"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command, default_samples in (("calibrate", 20), ("verify", 5)):
        sub = subparsers.add_parser(command)
        sub.add_argument("--side", choices=("left", "right"), required=True)
        sub.add_argument("--samples", type=int, default=default_samples)

    args = parser.parse_args()
    minimum = MIN_CALIBRATION_SAMPLES if args.command == "calibrate" else 3
    if args.samples < minimum:
        parser.error(f"{args.command} 至少需要 --samples {minimum}")
    try:
        ok = calibrate(args.side, args.samples) if args.command == "calibrate" else (
            verify(args.side, args.samples)
        )
    except KeyboardInterrupt:
        cprint("\n[hand-eye] 用户中断；未写入配置", "yellow")
        ok = False
    except (ImportError, KeyError, RuntimeError, ValueError, cv2.error) as exc:
        cprint(f"[hand-eye] STOP: {exc}", "red")
        ok = False
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
