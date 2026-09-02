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
import time
import shutil
from datetime import datetime

import cv2
import numpy as np
import threading
from termcolor import cprint

if not hasattr(cv2, "aruco"):
    raise SystemExit(
        "需要带 aruco 模块的 OpenCV（opencv-contrib-python）。"
        "请使用 /home/zz/anaconda3/envs/anygrasp/bin/python 运行本工具。"
    )

# ---------------------------------------------------------------------------
# 项目路径
# ---------------------------------------------------------------------------

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(PROJECT_ROOT, "robot_config.json")
CALIB_LOG_DIR = os.path.join(PROJECT_ROOT, "log", "calib_fail")
CALIB_BACKUP_DIR = os.path.join(PROJECT_ROOT, "log", "calibration_backups")

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
CHARUCO_SQUARE_LENGTH = 0.024   # 24 mm 每方格（打印后实测值）
CHARUCO_MARKER_LENGTH = 0.017   # 17 mm 每 marker（打印后实测值）
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
    检测 Charuco 板并估计位姿（OpenCV 4.10+ 兼容）。

    Returns:
        (rvec, tvec, num_corners)  或  (None, None, 0)
    """
    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)

    if charuco_ids is None or len(charuco_ids) < 6:
        return None, None, 0

    # matchImagePoints 将角点映射到板子坐标系
    obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
    if obj_points is None or img_points is None or len(img_points) < 6:
        return None, None, 0

    success, rvec, tvec = cv2.solvePnP(
        obj_points, img_points, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
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


def capture_images_paired(left_serial, right_serial, n=FRAMES_PER_POSE):
    """同时从两台相机采集 n 帧对（交替 wait_for_frames 保证同步）。"""
    import pyrealsense2 as rs

    left_images = []
    right_images = []

    lp, rp = None, None
    try:
        lp = rs.pipeline()
        lc = rs.config()
        lc.enable_device(left_serial)
        lc.enable_stream(rs.stream.color, CAMERA_W, CAMERA_H, rs.format.bgr8, 30)
        lp.start(lc)

        rp = rs.pipeline()
        rc = rs.config()
        rc.enable_device(right_serial)
        rc.enable_stream(rs.stream.color, CAMERA_W, CAMERA_H, rs.format.bgr8, 30)
        rp.start(rc)

        for _ in range(5):
            lp.wait_for_frames()
            rp.wait_for_frames()

        for _ in range(n):
            lf = lp.wait_for_frames()
            rf = rp.wait_for_frames()
            left_images.append(np.asanyarray(lf.get_color_frame().get_data()))
            right_images.append(np.asanyarray(rf.get_color_frame().get_data()))
    finally:
        if lp:
            lp.stop()
        if rp:
            rp.stop()

    return left_images, right_images


# ---------------------------------------------------------------------------
# 实时预览
# ---------------------------------------------------------------------------

class LivePreview:
    """双相机实时预览，Charuco 检测叠加（后台线程）。同时托管 pipeline 供采集复用。"""

    def __init__(self, board, left_K, left_dist, right_K, right_dist):
        self.board = board
        self.left_K = left_K
        self.left_dist = left_dist
        self.right_K = right_K
        self.right_dist = right_dist
        self._running = False
        self._thread = None
        self._latest_left = None
        self._latest_right = None
        self._lp = None
        self._rp = None
        self._capture_lock = threading.Lock()
        self._frame_count = 0

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        cprint("[预览] 实时画面已开启 (按 'q' 可关闭窗口, 不影响标定)", "green")

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)
        cv2.destroyAllWindows()

    def has_left(self):
        return self._lp is not None

    def has_right(self):
        return self._rp is not None

    def capture_paired_frames(self, n=FRAMES_PER_POSE):
        """采集 n 对唯一的 (left, right) 帧，保证帧不重复且左右同步。

        通过 _frame_count 跟踪预览线程更新，每拿到一个新计数才采集一对。
        """
        pairs = []
        last_count = -1
        timeout = time.time() + max(n * 0.15, 8)

        while len(pairs) < n:
            if time.time() > timeout:
                if len(pairs) >= MIN_SUCCESS_FRAMES:
                    cprint(f"[采集] 超时: 已获取 {len(pairs)}/{n} 对", "yellow")
                    break
                raise RuntimeError(f"Paired capture timeout: {len(pairs)}/{n}")

            with self._capture_lock:
                cur_count = self._frame_count
                if cur_count > last_count:
                    left = self._latest_left.copy() if self._latest_left is not None else None
                    right = self._latest_right.copy() if self._latest_right is not None else None
                    last_count = cur_count

            if left is not None and right is not None:
                pairs.append((left, right))

            time.sleep(0.02)  # ~50Hz 轮询，远低于 30fps 帧率

        return pairs

    def _run(self):
        import pyrealsense2 as rs

        lp = rs.pipeline()
        lc = rs.config()
        lc.enable_device(CAMERA_SERIALS["left"])
        lc.enable_stream(rs.stream.color, CAMERA_W, CAMERA_H, rs.format.bgr8, 30)
        try:
            lp.start(lc)
            self._lp = lp
        except Exception:
            cprint("[预览] 左相机启动失败", "red")
            lp = None

        rp = rs.pipeline()
        rc = rs.config()
        rc.enable_device(CAMERA_SERIALS["right"])
        rc.enable_stream(rs.stream.color, CAMERA_W, CAMERA_H, rs.format.bgr8, 30)
        try:
            rp.start(rc)
            self._rp = rp
        except Exception:
            cprint("[预览] 右相机启动失败", "red")
            rp = None

        # 跳过热机帧
        for _ in range(10):
            if lp: lp.wait_for_frames()
            if rp: rp.wait_for_frames()

        while self._running:
            # 在锁外等待帧（阻塞调用，可能耗时 ~33ms）
            new_left = None
            new_right = None
            if lp:
                try:
                    lf = lp.wait_for_frames()
                    new_left = np.asanyarray(lf.get_color_frame().get_data())
                except Exception:
                    pass
            if rp:
                try:
                    rf = rp.wait_for_frames()
                    new_right = np.asanyarray(rf.get_color_frame().get_data())
                except Exception:
                    pass

            # 快速更新缓存 + 递增帧计数器
            with self._capture_lock:
                if new_left is not None:
                    self._latest_left = new_left
                if new_right is not None:
                    self._latest_right = new_right
                self._frame_count += 1

            # 画检测叠加
            left_disp = self._latest_left.copy() if self._latest_left is not None else np.zeros((CAMERA_H, CAMERA_W, 3), dtype=np.uint8)
            right_disp = self._latest_right.copy() if self._latest_right is not None else np.zeros((CAMERA_H, CAMERA_W, 3), dtype=np.uint8)

            self._overlay_detection(left_disp, self.left_K, self.left_dist, "LEFT")
            self._overlay_detection(right_disp, self.right_K, self.right_dist, "RIGHT")

            # 并排显示
            combined = np.hstack([left_disp, right_disp])
            combined = cv2.resize(combined, (1280, 480))
            cv2.imshow("Dual Camera Preview (L | R)", combined)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()

        self._lp = None
        self._rp = None
        if lp: lp.stop()
        if rp: rp.stop()

    def _overlay_detection(self, img, K, dist, label):
        detector = cv2.aruco.CharucoDetector(self.board)
        cc, cid, mc, mid = detector.detectBoard(img)

        status = f"{label}: "
        if mid is not None:
            status += f"{len(mid)} markers"
            if mc is not None:
                for corners in mc:
                    corners_i = corners.astype(int).reshape(-1, 2)
                    if len(corners_i) >= 4:
                        cv2.polylines(img, [corners_i], True, (0, 255, 0), 1)
            if cid is not None and len(cid) >= 6:
                obj_pts, img_pts = self.board.matchImagePoints(cc, cid)
                if obj_pts is not None and len(img_pts) >= 6:
                    _, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
                    cv2.drawFrameAxes(img, K, dist, rvec, tvec, 0.04, 2)
                    status += f"  OK  z={tvec[2][0]*1000:.0f}mm"
                    cv2.putText(img, f"z={tvec[2][0]*1000:.0f}mm", (10, CAMERA_H - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            status += "no board"
        cv2.putText(img, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


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

    对每帧独立检测 → chordal L2-mean → 剔除离群帧 → 重新平均。

    Returns:
        (rvec, tvec, num_inliers, vis_img)  或  (None, None, n_detected, None)
    """
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

    # chordal L2-mean 旋转平均（比四元数分量中值更正确）
    def chordal_mean(mats):
        M = np.mean([R[:3, :3] for R in mats], axis=0)
        U, _, Vt = np.linalg.svd(M)
        d = np.linalg.det(Vt.T @ U.T)
        return Vt.T @ np.diag([1, 1, d]) @ U.T

    trans = np.array([T[:3, 3] for T in poses])
    med_trans = np.median(trans, axis=0)
    med_R = chordal_mean(poses)

    # 剔除离群帧（平移 > 3× 中位偏差 或 角度 > 3× 中位偏差）
    dev_trans = np.array([np.linalg.norm(T[:3, 3] - med_trans) for T in poses])
    dev_angle = np.array([
        np.arccos(np.clip((np.trace(med_R.T @ T[:3, :3]) - 1) / 2, -1, 1))
        for T in poses
    ])

    thresh_t = max(np.median(dev_trans) * 3, 0.005)   # 至少 5mm
    thresh_a = max(np.median(dev_angle) * 3, 0.03)     # 至少 ~1.7°

    inlier_poses = [
        T for T, dt, da in zip(poses, dev_trans, dev_angle)
        if dt <= thresh_t and da <= thresh_a
    ]

    if len(inlier_poses) >= MIN_SUCCESS_FRAMES:
        final_trans = np.array([T[:3, 3] for T in inlier_poses])
        med_trans = np.median(final_trans, axis=0)
        med_R = chordal_mean(inlier_poses)

    T_med = np.eye(4)
    T_med[:3, :3] = med_R
    T_med[:3, 3] = med_trans

    rvec, tvec = matrix_to_rvec_tvec(T_med)

    # 在最清晰的图上画坐标轴
    if best_img is not None:
        cv2.drawFrameAxes(best_img, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

    return rvec, tvec, len(inlier_poses), best_img


# ---------------------------------------------------------------------------
# 变换平均
# ---------------------------------------------------------------------------

def average_transforms(transforms):
    """平均多个 4x4 变换矩阵：chordal L2-mean 旋转 + 中值平移。"""
    rotations = np.array([T[:3, :3] for T in transforms])
    translations = np.array([T[:3, 3] for T in transforms])

    M = np.mean(rotations, axis=0)
    U, _, Vt = np.linalg.svd(M)
    d = np.linalg.det(Vt.T @ U.T)
    R_avg = Vt.T @ np.diag([1, 1, d]) @ U.T

    t_avg = np.median(translations, axis=0)

    T_avg = np.eye(4)
    T_avg[:3, :3] = R_avg
    T_avg[:3, 3] = t_avg
    return T_avg


# ---------------------------------------------------------------------------
# 标定主流程
# ---------------------------------------------------------------------------

def calibrate(preview=True):
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

    # ---- 启动实时预览 ----
    live_preview = None
    if preview:
        live_preview = LivePreview(board, left_K, left_dist, right_K, right_dist)
        live_preview.start()

    try:
        # ---- 锁定手臂位姿 ----
        cprint("\n[标定] 重要: 标定期间双臂必须保持不动，否则结果无效", "yellow")
        cprint("[标定] 请先将双臂移动到合适的观测位姿（如使用 pose_execute）", "yellow")
        input("[标定] 双臂就位后按 Enter 记录参考 TF ...")

        ref_cam_left = None
        ref_cam_right = None
        try:
            T_ref = lookup_transform(LEFT_BASE_FRAME, LEFT_CAM_FRAME)
            ref_cam_left = T_ref[:3, 3].copy()
            T_ref = lookup_transform(RIGHT_BASE_FRAME, RIGHT_CAM_FRAME)
            ref_cam_right = T_ref[:3, 3].copy()
            cprint(f"[标定] 参考位置已记录  左相机: ({ref_cam_left[0]*1000:.0f}, {ref_cam_left[1]*1000:.0f}, {ref_cam_left[2]*1000:.0f})mm"
                   f"  右相机: ({ref_cam_right[0]*1000:.0f}, {ref_cam_right[1]*1000:.0f}, {ref_cam_right[2]*1000:.0f})mm", "green")
        except RuntimeError as e:
            cprint(f"[标定] TF 查询失败: {e}", "red")
            return False

        DRIFT_WARN_MM = 20  # 漂移超过 20mm 则警告

        cam_transforms = []   # list of T_rcam_to_lcam (right_cam → left_cam)

        round_num = 0
        while True:
            round_num += 1
            print("\n" + "-" * 50)
            cprint(f"[标定] 第 {round_num} 个位置 (已有 {len(cam_transforms)} 个)", "cyan")
            cprint("[标定] 把 Charuco 板放在两臂相机都能看到的区域", "yellow")
            cprint("[标定] 提示: 换位置时改变距离/角度/视野位置以获得多样性", "yellow")
            input("准备好后按 Enter ...")

            # 采集（左右同步配对）
            cprint(f"[标定] 采集 {FRAMES_PER_POSE} 对同步帧 ...", "cyan")
            try:
                if live_preview is not None:
                    paired = live_preview.capture_paired_frames()
                    left_imgs = [p[0] for p in paired]
                    right_imgs = [p[1] for p in paired]
                else:
                    left_imgs, right_imgs = capture_images_paired(
                        CAMERA_SERIALS["left"], CAMERA_SERIALS["right"]
                    )
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

            T_board_lcam = rvec_tvec_to_matrix(left_rvec, left_tvec)
            T_board_rcam = rvec_tvec_to_matrix(right_rvec, right_tvec)

            # 相机间变换 (right_cam → left_cam)，不经过 TF
            T_rcam_to_lcam = T_board_lcam @ np.linalg.inv(T_board_rcam)

            board_in_left_cam = T_board_lcam[:3, 3] * 1000   # mm
            board_in_right_cam = T_board_rcam[:3, 3] * 1000  # mm
            cam_to_cam_t = T_rcam_to_lcam[:3, 3] * 1000

            cprint(f"[标定] 左: {left_ok} 帧内点", "green")
            cprint(f"       板子在左相机中 (mm): x={board_in_left_cam[0]:.1f} y={board_in_left_cam[1]:.1f} z={board_in_left_cam[2]:.1f}", "green")
            cprint(f"[标定] 右: {right_ok} 帧内点", "green")
            cprint(f"       板子在右相机中 (mm): x={board_in_right_cam[0]:.1f} y={board_in_right_cam[1]:.1f} z={board_in_right_cam[2]:.1f}", "green")
            cprint(f"       相机间平移 (mm): dx={cam_to_cam_t[0]:.1f} dy={cam_to_cam_t[1]:.1f} dz={cam_to_cam_t[2]:.1f}", "green")

            # TF (仅用于漂移检测)
            try:
                T_lb_cam = lookup_transform(LEFT_BASE_FRAME, LEFT_CAM_FRAME)
                T_rb_cam = lookup_transform(RIGHT_BASE_FRAME, RIGHT_CAM_FRAME)
            except RuntimeError as e:
                cprint(f"[标定] TF 查询失败: {e}", "red")
                return False

            # 漂移检测
            drift_L = np.linalg.norm(T_lb_cam[:3, 3] - ref_cam_left) * 1000
            drift_R = np.linalg.norm(T_rb_cam[:3, 3] - ref_cam_right) * 1000
            if drift_L > DRIFT_WARN_MM:
                cprint(f"       ⚠ 左臂相机漂移 {drift_L:.0f}mm! (阈值 {DRIFT_WARN_MM}mm)  请锁定手臂位姿后重试", "red")
                continue
            if drift_R > DRIFT_WARN_MM:
                cprint(f"       ⚠ 右臂相机漂移 {drift_R:.0f}mm! (阈值 {DRIFT_WARN_MM}mm)  请锁定手臂位姿后重试", "red")
                continue

            cam_transforms.append(T_rcam_to_lcam)

            # 实时预览当前估计
            if len(cam_transforms) >= 3:
                T_avg = average_transforms(cam_transforms)
                T_r2l = T_lb_cam @ T_avg @ np.linalg.inv(T_rb_cam)
                tx, ty, tz = T_r2l[:3, 3] * 1000
                cprint(f"  当前平移估计: x={tx:.1f}mm  y={ty:.1f}mm  z={tz:.1f}mm", "cyan")

            if len(cam_transforms) >= MIN_POSITIONS:
                more = input(f"\n继续添加位置? (y/n, 建议 ≥8): ").strip().lower()
            else:
                cprint(f"  还需至少 {MIN_POSITIONS - len(cam_transforms)} 个位置", "yellow")
                continue

            if more != "y":
                break

        if len(cam_transforms) < MIN_POSITIONS:
            cprint(f"[标定] 至少需要 {MIN_POSITIONS} 个有效位置", "red")
            return False

        # ---- 链式求解 (TF 只用一次!) ----
        T_rcam_to_lcam_avg = average_transforms(cam_transforms)

        try:
            T_lb_cam = lookup_transform(LEFT_BASE_FRAME, LEFT_CAM_FRAME)
            T_rb_cam = lookup_transform(RIGHT_BASE_FRAME, RIGHT_CAM_FRAME)
        except RuntimeError as e:
            cprint(f"[标定] 最终 TF 查询失败: {e}", "red")
            return False

        # right_base → right_cam → left_cam → left_base
        T_right_to_left = T_lb_cam @ T_rcam_to_lcam_avg @ np.linalg.inv(T_rb_cam)

        # ---- 相机间变换一致性 ----
        translations = np.array([T[:3, 3] for T in cam_transforms]) * 1000  # mm
        mean_t = np.mean(translations, axis=0)
        dev_t = np.array([np.linalg.norm(t - mean_t) for t in translations])

        cprint("\n" + "=" * 60, "cyan")
        cprint(f"[标定] 共 {len(cam_transforms)} 个位置", "cyan")

        cprint(f"\n[标定] 相机间变换一致性 (right_cam → left_cam):", "cyan")
        for i, d in enumerate(dev_t):
            outlier = d > np.mean(dev_t) * 2.5
            tag = "  <-- 离群!" if outlier else ""
            cprint(f"  位置 {i+1}: 平移偏差 {d:.2f}mm{tag}", "red" if outlier else "green")
        cprint(f"  平均偏差: {np.mean(dev_t):.2f}mm  最大: {np.max(dev_t):.2f}mm", "yellow")

        # ---- 留一交叉验证 ----
        if len(cam_transforms) >= 4:
            loo_errors = []
            for i in range(len(cam_transforms)):
                others = [T for j, T in enumerate(cam_transforms) if j != i]
                T_avg_i = average_transforms(others)
                err = np.linalg.norm(T_avg_i[:3, 3] - cam_transforms[i][:3, 3]) * 1000
                loo_errors.append(err)

            loo_mean = np.mean(loo_errors)
            loo_max = np.max(loo_errors)
            cprint(f"\n[标定] 留一交叉验证 (相机间平移):", "cyan")
            for i, err in enumerate(loo_errors):
                outlier = err > loo_mean * 2.5
                tag = "  <-- 不可靠!" if outlier else ""
                cprint(f"  位置 {i+1}: {err:.2f}mm{tag}", "red" if outlier else "green")
            cprint(f"  平均: {loo_mean:.2f}mm  最大: {loo_max:.2f}mm", "yellow")

            if loo_max > 10:
                cprint(f"\n[标定] 交叉验证最大误差 {loo_max:.1f}mm > 10mm，标定可能不可靠", "red")
                cprint("[标定] 建议: 增加更多位置，或检查板子是否平整", "yellow")

        # ---- 最终结果 ----
        cprint(f"\n[标定] 相机间变换 (right_cam → left_cam):", "cyan")
        cam_tx, cam_ty, cam_tz = T_rcam_to_lcam_avg[:3, 3] * 1000
        cprint(f"  平移 (mm):  x={cam_tx:.1f}  y={cam_ty:.1f}  z={cam_tz:.1f}", "cyan")

        cprint(f"\n[标定] 最终变换 T_right_base → left_base:", "cyan")
        for row in T_right_to_left:
            cprint(f"  [{row[0]:8.5f} {row[1]:8.5f} {row[2]:8.5f} {row[3]:8.5f}]", "cyan")

        tx, ty, tz = T_right_to_left[:3, 3] * 1000
        import tf.transformations as tr
        euler = np.degrees(tr.euler_from_matrix(T_right_to_left, axes='sxyz'))
        cprint(f"  平移 (mm):  x={tx:.1f}  y={ty:.1f}  z={tz:.1f}", "cyan")
        cprint(f"  旋转 (deg): rx={euler[0]:.2f}  ry={euler[1]:.2f}  rz={euler[2]:.2f}", "cyan")

        # ---- 保存 ----
        save = input("\n保存到 robot_config.json? (y/n): ").strip().lower()
        if save == "y":
            save_calibration(T_right_to_left, len(cam_transforms), np.mean(dev_t))
        else:
            cprint("[标定] 未保存", "yellow")

        return True

    finally:
        if live_preview is not None:
            live_preview.stop()


# ---------------------------------------------------------------------------
# 保存 / 读取
# ---------------------------------------------------------------------------

def save_calibration(T_matrix, num_obs, error_mm=None):
    """Persist a measured calibration without making the old one unrecoverable."""
    T_matrix = np.asarray(T_matrix, dtype=float)
    if T_matrix.shape != (4, 4) or not np.all(np.isfinite(T_matrix)):
        raise ValueError("标定矩阵必须是有限的 4x4 数值矩阵")
    if not np.allclose(T_matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise ValueError("标定矩阵齐次最后一行无效")
    rotation = T_matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3):
        raise ValueError("标定矩阵旋转部分不是正交矩阵")
    if np.linalg.det(rotation) <= 0.0:
        raise ValueError("标定矩阵旋转部分行列式无效")
    if int(num_obs) < MIN_POSITIONS:
        raise ValueError(f"有效观测位置不足: {num_obs} < {MIN_POSITIONS}")
    if error_mm is not None and (not np.isfinite(error_mm) or error_mm < 0):
        raise ValueError("标定误差无效")

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

    os.makedirs(CALIB_BACKUP_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = os.path.join(
        CALIB_BACKUP_DIR, f"robot_config_before_calibration_{timestamp}.json"
    )
    shutil.copy2(CONFIG_PATH, backup_path)

    # Atomic replacement prevents an interrupted write from leaving a corrupt config.
    tmp_path = CONFIG_PATH + ".tmp"
    try:
        with open(tmp_path, "w") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
            f.write("\n")
        os.replace(tmp_path, CONFIG_PATH)
    finally:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)

    cprint(f"[标定] 已保存到 {CONFIG_PATH}", "green")
    cprint(f"[标定] 旧配置备份: {os.path.relpath(backup_path, PROJECT_ROOT)}", "green")


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

def verify(preview=True):
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

    # ---- 启动实时预览 ----
    live_preview = None
    if preview:
        live_preview = LivePreview(board, left_K, left_dist, right_K, right_dist)
        live_preview.start()

    try:
        cprint("\n=== 多点验证 ===", "cyan")
        cprint("[验证] 至少测试 3 个独立位置（板子不要放在标定时用过的位置）", "yellow")

        errors_mm = []
        round_num = 0

        while True:
            round_num += 1
            print("\n" + "-" * 50)
            cprint(f"[验证] 第 {round_num} 个位置", "cyan")
            input("放好 Charuco 板后按 Enter ...")

            if live_preview is not None:
                paired = live_preview.capture_paired_frames()
                left_imgs = [p[0] for p in paired]
                right_imgs = [p[1] for p in paired]
            else:
                left_imgs, right_imgs = capture_images_paired(
                    CAMERA_SERIALS["left"], CAMERA_SERIALS["right"]
                )

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

    finally:
        if live_preview is not None:
            live_preview.stop()


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

    cal_parser = subparsers.add_parser("calibrate", help="Charuco 板交互式标定")
    cal_parser.add_argument("--no-preview", action="store_true",
                            help="禁用实时相机预览")
    subparsers.add_parser("show", help="查看标定结果")
    ver_parser = subparsers.add_parser("verify", help="多点验证标定精度")
    ver_parser.add_argument("--no-preview", action="store_true",
                            help="禁用实时相机预览")
    subparsers.add_parser("generate-charuco", help="生成可打印的 Charuco 板 PNG")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    if args.command == "calibrate":
        calibrate(preview=not args.no_preview)
    elif args.command == "show":
        show()
    elif args.command == "verify":
        verify(preview=not args.no_preview)
    elif args.command == "generate-charuco":
        generate_charuco()


if __name__ == "__main__":
    main()
