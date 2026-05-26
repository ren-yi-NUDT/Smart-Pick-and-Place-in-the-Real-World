#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂视觉标定工具 (v2 — Charuco 板)
=====================================
使用 Charuco 板标定左臂基座到右臂基座的刚体变换。

相比 v1（单 ArUco marker）的改进：
  - Charuco 板：几十个亚像素角点做 PnP，深度精度提升 5–10 倍
  - 时间平均：每位置采集 15 帧，中值滤波 + 离群帧剔除
  - SVD 刚体求解：Arun's method 最小二乘
  - 留一交叉验证：自动检测离群位置

使用：
  # 生成可打印的 Charuco 板
  python3 tools/calibrate_arms.py generate-charuco

  # 交互式标定（需要 ROS bringup 运行中）
  python3 tools/calibrate_arms.py calibrate

  # 查看已保存的标定结果
  python3 tools/calibrate_arms.py show

  # 多点验证标定精度
  python3 tools/calibrate_arms.py verify
"""

import os
import sys
import json
import argparse
from datetime import datetime

import cv2
import numpy as np
from termcolor import cprint

# ---------------------------------------------------------------------------
# 项目路径
# ---------------------------------------------------------------------------

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(PROJECT_ROOT, "robot_config.json")
CALIB_LOG_DIR = os.path.join(PROJECT_ROOT, "log", "calib_fail")

# ---------------------------------------------------------------------------
# 相机配置
# ---------------------------------------------------------------------------

CAMERA_SERIALS = {
    "left": "036322250517",
    "right": "242422304681",
}

CAMERA_W, CAMERA_H = 640, 480

# ---------------------------------------------------------------------------
# Charuco 板参数 (A4 可打印)
# ---------------------------------------------------------------------------

CHARUCO_SQUARES_X = 7
CHARUCO_SQUARES_Y = 9
CHARUCO_SQUARE_LENGTH = 0.022   # 22 mm 每方格
CHARUCO_MARKER_LENGTH = 0.016   # 16 mm 每 marker
CHARUCO_DICT = cv2.aruco.DICT_6X6_250

FRAMES_PER_POSE = 15          # 每位置采集帧数
MIN_SUCCESS_FRAMES = 8        # 最少成功检测帧数
MIN_POSITIONS = 3             # 最少标定位置数

# ---------------------------------------------------------------------------
# TF frame 名
# ---------------------------------------------------------------------------

LEFT_CAM_FRAME = "cam_link_grasp"
LEFT_BASE_FRAME = "base_link"
RIGHT_CAM_FRAME = "R_cam_link_grasp"
RIGHT_BASE_FRAME = "R_base_link"

# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------

def _save_debug_img(img, tag):
    os.makedirs(CALIB_LOG_DIR, exist_ok=True)
    ts = datetime.now().strftime("%H%M%S")
    path = os.path.join(CALIB_LOG_DIR, f"{tag}_{ts}.png")
    cv2.imwrite(path, img)
    cprint(f"[标定] 已保存: {path}", "yellow")


def _init_ros(name="calibrate_arms"):
    try:
        import rospy
        if not rospy.is_shutdown():
            try:
                rospy.init_node(name, anonymous=True, disable_signals=True)
            except rospy.exceptions.ROSException:
                pass
        return True
    except ImportError:
        return False


# ---------------------------------------------------------------------------
# Charuco 板
# ---------------------------------------------------------------------------

def create_charuco_board():
    return cv2.aruco.CharucoBoard(
        (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
        CHARUCO_SQUARE_LENGTH,
        CHARUCO_MARKER_LENGTH,
        cv2.aruco.getPredefinedDictionary(CHARUCO_DICT),
    )


def detect_charuco_pose(board, image, camera_matrix, dist_coeffs):
    """
    检测 Charuco 板并估计位姿。

    Returns:
        (rvec, tvec, num_corners)  或  (None, None, 0)
    """
    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)

    if charuco_ids is None or len(charuco_ids) < 4:
        return None, None, 0

    success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None
    )
    if not success:
        return None, None, 0

    return rvec.flatten(), tvec.flatten(), len(charuco_ids)


# ---------------------------------------------------------------------------
# 刚体变换工具
# ---------------------------------------------------------------------------

def rvec_tvec_to_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


def matrix_to_rvec_tvec(T):
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return rvec.flatten(), T[:3, 3]


# ---------------------------------------------------------------------------
# RealSense
# ---------------------------------------------------------------------------

def get_camera_intrinsics(serial):
    import pyrealsense2 as rs
    ctx = rs.context()
    for dev in ctx.query_devices():
        if dev.get_info(rs.camera_info.serial_number) == serial:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.color, CAMERA_W, CAMERA_H, rs.format.bgr8, 30)
            profile = pipeline.start(config)
            stream = profile.get_stream(rs.stream.color)
            intrinsics = stream.as_video_stream_profile().get_intrinsics()
            pipeline.stop()
            K = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1],
            ])
            return K, np.array(intrinsics.coeffs)
    raise RuntimeError(f"未找到序列号为 {serial} 的相机")


def capture_images(serial, n=FRAMES_PER_POSE):
    """连续采集 n 帧 BGR 图像"""
    import pyrealsense2 as rs
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, CAMERA_W, CAMERA_H, rs.format.bgr8, 30)
    pipeline.start(config)
    for _ in range(5):
        pipeline.wait_for_frames()

    images = []
    for _ in range(n):
        frames = pipeline.wait_for_frames()
        images.append(np.asanyarray(frames.get_color_frame().get_data()))

    pipeline.stop()
    return images


# ---------------------------------------------------------------------------
# ROS TF
# ---------------------------------------------------------------------------

def lookup_transform(from_frame, to_frame, timeout_s=5.0):
    import rospy
    import tf
    listener = tf.TransformListener()
    try:
        listener.waitForTransform(
            from_frame, to_frame, rospy.Time(), rospy.Duration(timeout_s),
        )
        (trans, rot) = listener.lookupTransform(from_frame, to_frame, rospy.Time())
    except Exception as e:
        raise RuntimeError(f"TF 查询失败 ({from_frame} → {to_frame}): {e}")
    T = np.eye(4)
    T[:3, :3] = tf.transformations.quaternion_matrix(rot)[:3, :3]
    T[:3, 3] = trans
    return T


# ---------------------------------------------------------------------------
# 时间平均位姿估计
# ---------------------------------------------------------------------------

def estimate_pose_robust(board, images, camera_matrix, dist_coeffs):
    """
    从多帧图像中鲁棒估计 Charuco 板位姿。

    对每帧独立检测 → 中值滤波 → 剔除离群帧 → 重新中值。

    Returns:
        (rvec, tvec, num_inliers, vis_img)  或  (None, None, n_detected, None)
    """
    import tf.transformations as tr

    poses = []          # list of 4x4
    best_img = None
    best_n = 0

    for img in images:
        rvec, tvec, n_corners = detect_charuco_pose(board, img, camera_matrix, dist_coeffs)
        if rvec is not None:
            poses.append(rvec_tvec_to_matrix(rvec, tvec))
            if n_corners > best_n:
                best_n = n_corners
                best_img = img.copy()

    if len(poses) < MIN_SUCCESS_FRAMES:
        return None, None, len(poses), best_img

    # 第一轮中值
    quats = np.array([tr.quaternion_from_matrix(T) for T in poses])
    trans = np.array([T[:3, 3] for T in poses])

    med_quat = np.median(quats, axis=0)
    med_quat /= np.linalg.norm(med_quat)
    med_trans = np.median(trans, axis=0)

    # 剔除离群帧（平移 > 3× 中位偏差 或 角度 > 3× 中位偏差）
    dev_trans = np.array([np.linalg.norm(T[:3, 3] - med_trans) for T in poses])
    dev_angle = np.array([
        2 * np.arccos(min(1, abs(np.dot(tr.quaternion_from_matrix(T), med_quat))))
        for T in poses
    ])

    thresh_t = max(np.median(dev_trans) * 3, 0.005)   # 至少 5mm
    thresh_a = max(np.median(dev_angle) * 3, 0.03)     # 至少 ~1.7°

    inlier_poses = [
        T for T, dt, da in zip(poses, dev_trans, dev_angle)
        if dt <= thresh_t and da <= thresh_a
    ]

    if len(inlier_poses) >= MIN_SUCCESS_FRAMES:
        final_quats = np.array([tr.quaternion_from_matrix(T) for T in inlier_poses])
        final_trans = np.array([T[:3, 3] for T in inlier_poses])
        med_quat = np.median(final_quats, axis=0)
        med_quat /= np.linalg.norm(med_quat)
        med_trans = np.median(final_trans, axis=0)

    T_med = tr.quaternion_matrix(med_quat)
    T_med[:3, 3] = med_trans

    rvec, tvec = matrix_to_rvec_tvec(T_med)

    # 在最清晰的图上画坐标轴
    if best_img is not None:
        cv2.drawFrameAxes(best_img, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

    return rvec, tvec, len(inlier_poses if 'inlier_poses' in dir() else poses), best_img


# ---------------------------------------------------------------------------
# SVD 刚体变换求解 (Arun's method)
# ---------------------------------------------------------------------------

def solve_rigid_transform_svd(points_left, points_right):
    """
    最小化  sum_i ||p_L_i - (R * p_R_i + t)||^2

    Args:
        points_left:  (N, 3) — 左臂基座坐标
        points_right: (N, 3) — 右臂基座坐标

    Returns:
        T_right_to_left: 4×4 变换矩阵
    """
    c_L = np.mean(points_left, axis=0)
    c_R = np.mean(points_right, axis=0)

    q_L = points_left - c_L
    q_R = points_right - c_R

    H = q_L.T @ q_R                     # 3×3 互协方差

    U, S, Vt = np.linalg.svd(H)
    V = Vt.T

    R = V @ U.T
    if np.linalg.det(R) < 0:            # 反射修正
        V[:, -1] *= -1
        R = V @ U.T

    t = c_L - R @ c_R

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# ---------------------------------------------------------------------------
# 留一交叉验证
# ---------------------------------------------------------------------------

def leave_one_out_errors(points_left, points_right):
    """
    每次留一个点对做验证，其余做训练。

    Returns:
        errors_mm: 每位置误差 (mm)
        mean_mm, max_mm
    """
    n = len(points_left)
    if n < 3:
        return [], None, None

    errors_mm = []
    for i in range(n):
        mask = np.ones(n, dtype=bool)
        mask[i] = False
        T = solve_rigid_transform_svd(points_left[mask], points_right[mask])
        pred = T[:3, :3] @ points_right[i] + T[:3, 3]
        err = np.linalg.norm(pred - points_left[i]) * 1000
        errors_mm.append(err)

    return errors_mm, np.mean(errors_mm), np.max(errors_mm)


# ---------------------------------------------------------------------------
# 标定主流程
# ---------------------------------------------------------------------------

def calibrate():
    board = create_charuco_board()

    cprint("\n=== 双臂视觉标定 (Charuco 板) ===", "cyan")
    cprint(f"板子: {CHARUCO_SQUARES_X}×{CHARUCO_SQUARES_Y} 方格, "
           f"{CHARUCO_SQUARE_LENGTH*1000:.0f}mm/格, "
           f"marker {CHARUCO_MARKER_LENGTH*1000:.0f}mm", "yellow")
    cprint(f"字典: DICT_6X6_250", "yellow")
    cprint(f"采集: {FRAMES_PER_POSE} 帧/位置, 最少 {MIN_SUCCESS_FRAMES} 帧成功", "yellow")

    if not _init_ros():
        cprint("[标定] 需要 ROS 环境，请先启动 bringup", "red")
        return False

    cprint("\n[标定] 获取相机内参...", "cyan")
    try:
        left_K, left_dist = get_camera_intrinsics(CAMERA_SERIALS["left"])
        right_K, right_dist = get_camera_intrinsics(CAMERA_SERIALS["right"])
        cprint("[标定] 相机内参获取成功", "green")
    except Exception as e:
        cprint(f"[标定] 获取相机内参失败: {e}", "red")
        return False

    pts_L = []   # (N, 3) 板子原点 → 左臂基座
    pts_R = []   # (N, 3) 板子原点 → 右臂基座

    round_num = 0
    while True:
        round_num += 1
        print("\n" + "-" * 50)
        cprint(f"[标定] 第 {round_num} 个位置 (已有 {len(pts_L)} 个)", "cyan")
        cprint("[标定] 把 Charuco 板放在两臂相机都能看到的区域", "yellow")
        cprint("[标定] 提示: 换位置时改变距离/角度/视野位置以获得多样性", "yellow")
        input("准备好后按 Enter ...")

        # 采集
        cprint(f"[标定] 采集 {FRAMES_PER_POSE} 帧 ...", "cyan")
        try:
            left_imgs = capture_images(CAMERA_SERIALS["left"])
            right_imgs = capture_images(CAMERA_SERIALS["right"])
        except Exception as e:
            cprint(f"[标定] 采集失败: {e}", "red")
            continue

        # 位姿估计
        cprint("[标定] 检测左相机 ...", "cyan")
        left_rvec, left_tvec, left_ok, left_vis = estimate_pose_robust(
            board, left_imgs, left_K, left_dist,
        )
        cprint("[标定] 检测右相机 ...", "cyan")
        right_rvec, right_tvec, right_ok, right_vis = estimate_pose_robust(
            board, right_imgs, right_K, right_dist,
        )

        if left_rvec is None:
            cprint(f"[标定] 左相机检测失败 ({left_ok}/{FRAMES_PER_POSE} 帧成功, "
                   f"需要 ≥{MIN_SUCCESS_FRAMES})", "red")
            if left_vis is not None:
                _save_debug_img(left_vis, f"left_fail_r{round_num}")
            continue
        if right_rvec is None:
            cprint(f"[标定] 右相机检测失败 ({right_ok}/{FRAMES_PER_POSE} 帧成功, "
                   f"需要 ≥{MIN_SUCCESS_FRAMES})", "red")
            if right_vis is not None:
                _save_debug_img(right_vis, f"right_fail_r{round_num}")
            continue

        if left_vis is not None:
            _save_debug_img(left_vis, f"left_detect_r{round_num}")
        if right_vis is not None:
            _save_debug_img(right_vis, f"right_detect_r{round_num}")

        cprint(f"[标定] 左: {left_ok} 帧内点, t={left_tvec.round(4)}", "green")
        cprint(f"[标定] 右: {right_ok} 帧内点, t={right_tvec.round(4)}", "green")

        # TF
        try:
            T_lb_cam = lookup_transform(LEFT_BASE_FRAME, LEFT_CAM_FRAME)
            T_rb_cam = lookup_transform(RIGHT_BASE_FRAME, RIGHT_CAM_FRAME)
        except RuntimeError as e:
            cprint(f"[标定] TF 查询失败: {e}", "red")
            return False

        # 板子原点 → 各臂基座
        board_in_left_base = (T_lb_cam @ rvec_tvec_to_matrix(left_rvec, left_tvec))[:3, 3]
        board_in_right_base = (T_rb_cam @ rvec_tvec_to_matrix(right_rvec, right_tvec))[:3, 3]

        pts_L.append(board_in_left_base)
        pts_R.append(board_in_right_base)

        # 实时预览当前估计
        if len(pts_L) >= 3:
            T_cur = solve_rigid_transform_svd(np.array(pts_L), np.array(pts_R))
            tx, ty, tz = T_cur[:3, 3] * 1000
            cprint(f"  当前平移估计: x={tx:.1f}mm  y={ty:.1f}mm  z={tz:.1f}mm", "cyan")

        if len(pts_L) >= MIN_POSITIONS:
            more = input(f"\n继续添加位置? (y/n, 建议 ≥8): ").strip().lower()
        else:
            cprint(f"  还需至少 {MIN_POSITIONS - len(pts_L)} 个位置", "yellow")
            continue

        if more != "y":
            break

    if len(pts_L) < MIN_POSITIONS:
        cprint(f"[标定] 至少需要 {MIN_POSITIONS} 个有效位置", "red")
        return False

    # ---- SVD 求解 ----
    pts_L = np.array(pts_L)
    pts_R = np.array(pts_R)
    T_final = solve_rigid_transform_svd(pts_L, pts_R)

    # ---- 拟合残差 ----
    residuals = []
    for i in range(len(pts_L)):
        pred = T_final[:3, :3] @ pts_R[i] + T_final[:3, 3]
        err = np.linalg.norm(pred - pts_L[i]) * 1000
        residuals.append(err)

    mean_res = np.mean(residuals)
    max_res = np.max(residuals)

    cprint("\n" + "=" * 60, "cyan")
    cprint(f"[标定] 共 {len(pts_L)} 个位置", "cyan")

    cprint(f"\n[标定] 拟合残差:", "cyan")
    for i, err in enumerate(residuals):
        outlier = err > mean_res * 2.5
        tag = "  <-- 离群!" if outlier else ""
        cprint(f"  位置 {i+1}: {err:.2f}mm{tag}", "red" if outlier else "green")
    cprint(f"  平均: {mean_res:.2f}mm  最大: {max_res:.2f}mm", "yellow")

    # ---- 留一交叉验证 ----
    loo_errors, loo_mean, loo_max = leave_one_out_errors(pts_L, pts_R)
    if loo_errors is not None:
        cprint(f"\n[标定] 留一交叉验证:", "cyan")
        for i, err in enumerate(loo_errors):
            outlier = err > loo_mean * 2.5
            tag = "  <-- 不可靠!" if outlier else ""
            cprint(f"  位置 {i+1}: {err:.2f}mm{tag}", "red" if outlier else "green")
        cprint(f"  平均: {loo_mean:.2f}mm  最大: {loo_max:.2f}mm", "yellow")

        if loo_max > 15:
            cprint(f"\n[标定] 交叉验证最大误差 {loo_max:.1f}mm > 15mm，标定可能不可靠", "red")
            cprint("[标定] 建议: 增加更多位置，或检查板子是否平整、TF 是否准确", "yellow")

    # ---- 最终结果 ----
    cprint(f"\n[标定] 最终变换 T_right_base → left_base:", "cyan")
    for row in T_final:
        cprint(f"  [{row[0]:8.5f} {row[1]:8.5f} {row[2]:8.5f} {row[3]:8.5f}]", "cyan")

    tx, ty, tz = T_final[:3, 3] * 1000
    import tf.transformations as tr
    euler = np.degrees(tr.euler_from_matrix(T_final, axes='sxyz'))
    cprint(f"  平移 (mm):  x={tx:.1f}  y={ty:.1f}  z={tz:.1f}", "cyan")
    cprint(f"  旋转 (deg): rx={euler[0]:.2f}  ry={euler[1]:.2f}  rz={euler[2]:.2f}", "cyan")

    # ---- 保存 ----
    save = input("\n保存到 robot_config.json? (y/n): ").strip().lower()
    if save == "y":
        save_calibration(T_final, len(pts_L), mean_res)
    else:
        cprint("[标定] 未保存", "yellow")

    return True


# ---------------------------------------------------------------------------
# 保存 / 读取
# ---------------------------------------------------------------------------

def save_calibration(T_matrix, num_obs, error_mm=None):
    with open(CONFIG_PATH, "r") as f:
        config = json.load(f)

    calib = {
        "T_right_to_left": T_matrix.tolist(),
        "method": "charuco",
        "num_observations": num_obs,
        "timestamp": datetime.now().isoformat(),
    }
    if error_mm is not None:
        calib["mean_error_mm"] = round(error_mm, 2)

    config.setdefault("shared", {})["calibration"] = calib

    with open(CONFIG_PATH, "w") as f:
        json.dump(config, f, indent=2, ensure_ascii=False)

    cprint(f"[标定] 已保存到 {CONFIG_PATH}", "green")


def load_calibration():
    with open(CONFIG_PATH, "r") as f:
        config = json.load(f)
    return config.get("shared", {}).get("calibration")


# ---------------------------------------------------------------------------
# 查看
# ---------------------------------------------------------------------------

def show():
    calib = load_calibration()
    if calib is None:
        cprint("[标定] 尚未标定，运行 calibrate 命令开始标定", "yellow")
        return

    T = np.array(calib["T_right_to_left"])
    cprint("\n=== 双臂标定结果 ===", "cyan")
    cprint(f"  时间:      {calib.get('timestamp', 'N/A')}", "cyan")
    cprint(f"  方法:      {calib.get('method', 'unknown')}", "cyan")
    cprint(f"  观测次数:  {calib.get('num_observations', 'N/A')}", "cyan")
    if "mean_error_mm" in calib:
        cprint(f"  拟合误差:  {calib['mean_error_mm']}mm", "cyan")

    cprint("\n  T_right_to_left:", "cyan")
    for row in T:
        cprint(f"    [{row[0]:8.5f} {row[1]:8.5f} {row[2]:8.5f} {row[3]:8.5f}]", "cyan")

    cprint(f"\n  平移 (mm): x={T[0,3]*1000:.1f}  y={T[1,3]*1000:.1f}  z={T[2,3]*1000:.1f}", "cyan")

    import tf.transformations as tr
    euler = np.degrees(tr.euler_from_matrix(T, axes='sxyz'))
    cprint(f"  旋转 (deg): rx={euler[0]:.2f}  ry={euler[1]:.2f}  rz={euler[2]:.2f}", "cyan")


# ---------------------------------------------------------------------------
# 多点验证
# ---------------------------------------------------------------------------

def verify():
    calib = load_calibration()
    if calib is None:
        cprint("[标定] 尚未标定", "red")
        return

    T_r2l = np.array(calib["T_right_to_left"])

    if not _init_ros("calibrate_arms_verify"):
        cprint("[标定] 需要 ROS 环境", "red")
        return

    board = create_charuco_board()
    left_K, left_dist = get_camera_intrinsics(CAMERA_SERIALS["left"])
    right_K, right_dist = get_camera_intrinsics(CAMERA_SERIALS["right"])

    cprint("\n=== 多点验证 ===", "cyan")
    cprint("[验证] 至少测试 3 个独立位置（板子不要放在标定时用过的位置）", "yellow")

    errors_mm = []
    round_num = 0

    while True:
        round_num += 1
        print("\n" + "-" * 50)
        cprint(f"[验证] 第 {round_num} 个位置", "cyan")
        input("放好 Charuco 板后按 Enter ...")

        left_imgs = capture_images(CAMERA_SERIALS["left"])
        right_imgs = capture_images(CAMERA_SERIALS["right"])

        left_rvec, left_tvec, left_ok, _ = estimate_pose_robust(
            board, left_imgs, left_K, left_dist,
        )
        right_rvec, right_tvec, right_ok, _ = estimate_pose_robust(
            board, right_imgs, right_K, right_dist,
        )

        if left_rvec is None or right_rvec is None:
            cprint(f"[验证] 检测失败 (左:{left_ok} 右:{right_ok})", "red")
            continue

        try:
            T_lb_cam = lookup_transform(LEFT_BASE_FRAME, LEFT_CAM_FRAME)
            T_rb_cam = lookup_transform(RIGHT_BASE_FRAME, RIGHT_CAM_FRAME)
        except RuntimeError as e:
            cprint(f"[验证] TF 查询失败: {e}", "red")
            continue

        # 直接观测 vs 通过标定变换预测
        board_in_left_base_direct = (T_lb_cam @ rvec_tvec_to_matrix(left_rvec, left_tvec))[:3, 3]
        board_in_right_base = (T_rb_cam @ rvec_tvec_to_matrix(right_rvec, right_tvec))[:3, 3]
        board_in_left_base_pred = T_r2l[:3, :3] @ board_in_right_base + T_r2l[:3, 3]

        err = np.linalg.norm(board_in_left_base_direct - board_in_left_base_pred) * 1000
        errors_mm.append(err)

        color = "green" if err < 5 else ("yellow" if err < 10 else "red")
        cprint(f"[验证] 位置 {round_num} 误差: {err:.2f}mm", color)

        if round_num >= 3:
            more = input("\n继续验证? (y/n): ").strip().lower()
            if more != "y":
                break

    if not errors_mm:
        return

    mean_e = np.mean(errors_mm)
    max_e = np.max(errors_mm)
    min_e = np.min(errors_mm)

    cprint(f"\n[验证] {'─' * 40}", "cyan")
    cprint(f"  位置数: {len(errors_mm)}", "cyan")
    cprint(f"  平均: {mean_e:.2f}mm  最大: {max_e:.2f}mm  最小: {min_e:.2f}mm", "yellow")

    if mean_e < 5:
        cprint("  结论: 精度良好 (<5mm)，可用于双臂协作", "green")
    elif mean_e < 10:
        cprint("  结论: 精度一般 (5-10mm)，仅适用于较大物体", "yellow")
    else:
        cprint("  结论: 精度不足 (>10mm)，建议重新标定", "red")


# ---------------------------------------------------------------------------
# 生成 Charuco 板
# ---------------------------------------------------------------------------

def generate_charuco():
    board = create_charuco_board()

    board_w_mm = CHARUCO_SQUARES_X * CHARUCO_SQUARE_LENGTH * 1000
    board_h_mm = CHARUCO_SQUARES_Y * CHARUCO_SQUARE_LENGTH * 1000

    # ~200 DPI 适合打印
    dpi = 200
    px_per_mm = dpi / 25.4
    margin_mm = 15

    out_w = int((board_w_mm + 2 * margin_mm) * px_per_mm)
    out_h = int((board_h_mm + 2 * margin_mm) * px_per_mm)
    margin_px = int(margin_mm * px_per_mm)

    board_img = board.generateImage((out_w, out_h), marginSize=margin_px, borderBits=1)

    output_path = os.path.join(PROJECT_ROOT, "tools", "charuco_board.png")
    cv2.imwrite(output_path, board_img)

    cprint(f"\n  Charuco 板已生成: {os.path.relpath(output_path, PROJECT_ROOT)}", "green")
    cprint(f"  参数: {CHARUCO_SQUARES_X}×{CHARUCO_SQUARES_Y} 方格, "
           f"{CHARUCO_SQUARE_LENGTH*1000:.0f}mm/格, "
           f"marker {CHARUCO_MARKER_LENGTH*1000:.0f}mm", "green")
    cprint(f"  板子尺寸: {board_w_mm:.0f}×{board_h_mm:.0f}mm (不含边框)", "green")
    cprint(f"  图像: {out_w}×{out_h}px @ ~{dpi}DPI", "green")
    cprint(f"\n  打印说明:", "yellow")
    cprint(f"    1. A4 纸 100% 比例打印 (禁止缩放/适应页面)", "yellow")
    cprint(f"    2. 贴在硬纸板或泡沫板上保证平整", "yellow")
    cprint(f"    3. 打印后实测方格边长，应为 {CHARUCO_SQUARE_LENGTH*1000:.0f}mm", "yellow")
    cprint(f"       如有偏差请修改脚本顶部 CHARUCO_SQUARE_LENGTH", "yellow")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="双臂视觉标定工具 (v2 — Charuco 板)")
    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("calibrate", help="Charuco 板交互式标定")
    subparsers.add_parser("show", help="查看标定结果")
    subparsers.add_parser("verify", help="多点验证标定精度")
    subparsers.add_parser("generate-charuco", help="生成可打印的 Charuco 板 PNG")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    if args.command == "calibrate":
        calibrate()
    elif args.command == "show":
        show()
    elif args.command == "verify":
        verify()
    elif args.command == "generate-charuco":
        generate_charuco()


if __name__ == "__main__":
    main()
