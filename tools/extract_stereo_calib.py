#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双臂立体标定工具 (Charuco 板 — 拍照模式)
=========================================
不依赖 ROS camera_calibration 视频流，直接用 pyrealsense2 抓帧，
用 Charuco 板做立体标定，结合 ROS TF 提取双臂基座间变换。

适用于 USB 2.0 等无法稳定发布 ROS topic 的场景。

使用：
    # 1. 先启动 bringup（提供 TF）
    # 2. 运行本脚本
    python3 tools/extract_stereo_calib.py calibrate
    python3 tools/extract_stereo_calib.py show
    python3 tools/extract_stereo_calib.py verify
"""

import os
import sys
import json
import argparse
import time
import threading
from datetime import datetime

import cv2
import numpy as np
from termcolor import cprint

# ---------------------------------------------------------------------------
# 项目路径
# ---------------------------------------------------------------------------

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(PROJECT_ROOT, "robot_config.json")
CALIB_LOG_DIR = os.path.join(PROJECT_ROOT, "log", "calib_stereo")

# ---------------------------------------------------------------------------
# 相机配置（与 calibrate_arms.py 一致）
# ---------------------------------------------------------------------------

CAMERA_SERIALS = {
    "left": "036322250517",
    "right": "242422304681",
}

CAMERA_W, CAMERA_H = 640, 480

# ---------------------------------------------------------------------------
# Charuco 板参数（与 calibrate_arms.py 完全一致）
# ---------------------------------------------------------------------------

CHARUCO_SQUARES_X = 7
CHARUCO_SQUARES_Y = 9
CHARUCO_SQUARE_LENGTH = 0.024   # 24 mm
CHARUCO_MARKER_LENGTH = 0.017   # 17 mm
CHARUCO_DICT = cv2.aruco.DICT_6X6_250

# ---------------------------------------------------------------------------
# TF frame 名（与 calibrate_arms.py 一致）
# ---------------------------------------------------------------------------

LEFT_CAM_FRAME = "cam_link_grasp"
LEFT_BASE_FRAME = "base_link"
RIGHT_CAM_FRAME = "R_cam_link_grasp"
RIGHT_BASE_FRAME = "R_base_link"

MIN_PAIRS = 12              # 最少图像对数


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


def detect_charuco(board, image):
    """检测 Charuco 板，返回 (corners_2d, obj_points_3d, ids, num_corners)"""
    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)

    if charuco_ids is None or len(charuco_ids) < 6:
        return None, None, None, 0

    obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
    if obj_points is None or img_points is None or len(img_points) < 6:
        return None, None, None, 0

    return img_points, obj_points, charuco_ids, len(charuco_ids)


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
            config.enable_stream(rs.stream.color, CAMERA_W, CAMERA_H, rs.format.bgr8, 15)
            profile = pipeline.start(config)
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            pipeline.stop()
            K = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1],
            ])
            return K, np.array(intrinsics.coeffs)
    raise RuntimeError(f"未找到序列号为 {serial} 的相机")


# ---------------------------------------------------------------------------
# 实时预览
# ---------------------------------------------------------------------------

class LivePreview:
    """双相机实时预览 + Charuco 检测叠加（后台线程）。
    同时托管 pipeline，支持 grab_frames() 即时抓帧。"""

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
        self._lock = threading.Lock()
        self._frame_count = 0

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        cprint("[预览] 实时画面已开启 (按 'q' 关闭窗口不影响标定)", "green")

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)
        cv2.destroyAllWindows()

    def grab_frames(self):
        """从预览流中立即返回最新的一对帧 (left, right)"""
        with self._lock:
            left = self._latest_left.copy() if self._latest_left is not None else None
            right = self._latest_right.copy() if self._latest_right is not None else None
        return left, right

    def _run(self):
        import pyrealsense2 as rs

        lp = rp = None
        try:
            lp = rs.pipeline()
            lc = rs.config()
            lc.enable_device(CAMERA_SERIALS["left"])
            lc.enable_stream(rs.stream.color, CAMERA_W, CAMERA_H, rs.format.bgr8, 15)
            lp.start(lc)
        except Exception:
            cprint("[预览] 左相机启动失败", "red")
            lp = None

        try:
            rp = rs.pipeline()
            rc = rs.config()
            rc.enable_device(CAMERA_SERIALS["right"])
            rc.enable_stream(rs.stream.color, CAMERA_W, CAMERA_H, rs.format.bgr8, 15)
            rp.start(rc)
        except Exception:
            cprint("[预览] 右相机启动失败", "red")
            rp = None

        # 热身
        for _ in range(10):
            if lp: lp.wait_for_frames()
            if rp: rp.wait_for_frames()

        while self._running:
            new_left = new_right = None
            if lp:
                try:
                    new_left = np.asanyarray(lp.wait_for_frames().get_color_frame().get_data())
                except Exception:
                    pass
            if rp:
                try:
                    new_right = np.asanyarray(rp.wait_for_frames().get_color_frame().get_data())
                except Exception:
                    pass

            with self._lock:
                if new_left is not None:
                    self._latest_left = new_left
                if new_right is not None:
                    self._latest_right = new_right
                self._frame_count += 1

            # 画检测叠加
            left_disp = self._latest_left.copy() if self._latest_left is not None \
                else np.zeros((CAMERA_H, CAMERA_W, 3), dtype=np.uint8)
            right_disp = self._latest_right.copy() if self._latest_right is not None \
                else np.zeros((CAMERA_H, CAMERA_W, 3), dtype=np.uint8)

            self._overlay(left_disp, self.left_K, self.left_dist, "LEFT")
            self._overlay(right_disp, self.right_K, self.right_dist, "RIGHT")

            combined = np.hstack([left_disp, right_disp])
            combined = cv2.resize(combined, (1280, 480))
            cv2.imshow("Stereo Calib Preview (L | R)", combined)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()

        if lp: lp.stop()
        if rp: rp.stop()

    def _overlay(self, img, K, dist, label):
        detector = cv2.aruco.CharucoDetector(self.board)
        cc, cid, mc, mid = detector.detectBoard(img)

        status = f"{label}: "
        if mid is not None:
            status += f"{len(mid)} markers"
            if mc is not None:
                for corners in mc:
                    pts = corners.astype(int).reshape(-1, 2)
                    if len(pts) >= 4:
                        cv2.polylines(img, [pts], True, (0, 255, 0), 1)
            if cid is not None and len(cid) >= 6:
                obj_pts, img_pts = self.board.matchImagePoints(cc, cid)
                if obj_pts is not None and len(img_pts) >= 6:
                    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist,
                                                   flags=cv2.SOLVEPNP_ITERATIVE)
                    if ok:
                        cv2.drawFrameAxes(img, K, dist, rvec, tvec, 0.04, 2)
                        z_mm = tvec[2][0] * 1000
                        status += f"  OK  z={z_mm:.0f}mm  corners={len(cid)}"
                        cv2.putText(img, f"z={z_mm:.0f}mm", (10, CAMERA_H - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            status += "no board"
        cv2.putText(img, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


# ---------------------------------------------------------------------------
# ROS TF
# ---------------------------------------------------------------------------

def _init_ros(name="extract_stereo_calib"):
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
# 保存 / 读取
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


def save_calibration(T_matrix, num_obs, error_mm=None):
    with open(CONFIG_PATH, "r") as f:
        config = json.load(f)

    calib = {
        "T_right_to_left": T_matrix.tolist(),
        "method": "charuco_svd_direct",
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


def _save_debug_img(img, tag):
    os.makedirs(CALIB_LOG_DIR, exist_ok=True)
    ts = datetime.now().strftime("%H%M%S")
    path = os.path.join(CALIB_LOG_DIR, f"{tag}_{ts}.png")
    cv2.imwrite(path, img)
    cprint(f"[标定] 已保存: {path}", "yellow")


# ---------------------------------------------------------------------------
# 标定主流程
# ---------------------------------------------------------------------------

def calibrate():
    board = create_charuco_board()

    cprint("\n=== 双臂立体标定 (Charuco 板 — 拍照模式) ===", "cyan")
    cprint(f"板子: {CHARUCO_SQUARES_X}×{CHARUCO_SQUARES_Y} 方格, "
           f"{CHARUCO_SQUARE_LENGTH*1000:.0f}mm/格, "
           f"marker {CHARUCO_MARKER_LENGTH*1000:.0f}mm", "yellow")
    cprint(f"字典: DICT_6X6_250", "yellow")

    # ---- 获取相机内参 ----
    cprint("\n[标定] 获取相机内参 ...", "cyan")
    try:
        left_K, left_dist = get_camera_intrinsics(CAMERA_SERIALS["left"])
        right_K, right_dist = get_camera_intrinsics(CAMERA_SERIALS["right"])
        cprint("[标定] 相机内参获取成功", "green")
    except Exception as e:
        cprint(f"[标定] 获取相机内参失败: {e}", "red")
        return False

    # ---- 初始化 ROS ----
    if not _init_ros():
        cprint("[标定] 需要 ROS 环境，请先启动 bringup", "red")
        return False

    # ---- 锁定手臂位姿 ----
    cprint("\n[标定] 重要: 标定期间双臂必须保持不动，否则结果无效", "yellow")
    input("[标定] 双臂就位后按 Enter 记录参考 TF ...")

    try:
        T_lb_cam = lookup_transform(LEFT_BASE_FRAME, LEFT_CAM_FRAME)
        T_rb_cam = lookup_transform(RIGHT_BASE_FRAME, RIGHT_CAM_FRAME)
    except RuntimeError as e:
        cprint(f"[标定] TF 查询失败: {e}", "red")
        cprint("[提示] 请确保 bringup 正在运行", "yellow")
        return False

    ref_left = T_lb_cam[:3, 3].copy()
    ref_right = T_rb_cam[:3, 3].copy()
    cprint(f"[标定] 参考位置  左相机: ({ref_left[0]*1000:.0f}, {ref_left[1]*1000:.0f}, {ref_left[2]*1000:.0f})mm"
           f"  右相机: ({ref_right[0]*1000:.0f}, {ref_right[1]*1000:.0f}, {ref_right[2]*1000:.0f})mm", "green")

    # ---- 启动实时预览 ----
    preview = LivePreview(board, left_K, left_dist, right_K, right_dist)
    preview.start()

    try:
        # ---- 采集：每位置收集板子在两基座中的坐标 ----
        left_points_base = []    # list of 3x1: board origin in left_base frame
        right_points_base = []   # list of 3x1: board origin in right_base frame

        pair_num = 0
        while True:
            pair_num += 1
            print("\n" + "-" * 50)
            cprint(f"[标定] 第 {pair_num} 个位置 (已有 {len(left_points_base)} 对有效数据)", "cyan")
            cprint("[标定] 把 Charuco 板放在两臂相机都能看到的区域", "yellow")
            cprint("[标定] 提示: 变换距离/角度/位置以获得多样性", "yellow")
            input("准备好后按 Enter 拍照 ...")

            left_img, right_img = preview.grab_frames()
            if left_img is None or right_img is None:
                cprint("[标定] 相机帧未就绪，请稍等", "red")
                continue

            l_pts, l_obj, l_ids, l_n = detect_charuco(board, left_img)
            r_pts, r_obj, r_ids, r_n = detect_charuco(board, right_img)

            if l_pts is None or r_pts is None:
                cprint(f"[标定] 检测失败 (左:{l_n} 右:{r_n})，需要两边都检测到", "red")
                _save_debug_img(left_img, f"left_fail_p{pair_num}")
                _save_debug_img(right_img, f"right_fail_p{pair_num}")
                continue

            # 公共 id 精确匹配
            common_ids = np.intersect1d(l_ids.flatten(), r_ids.flatten())
            if len(common_ids) < 6:
                cprint(f"[标定] 公共角点仅 {len(common_ids)} 个 (需 ≥6)，跳过", "red")
                continue

            l_id_to_idx = {int(id_): i for i, id_ in enumerate(l_ids.flatten())}
            r_id_to_idx = {int(id_): i for i, id_ in enumerate(r_ids.flatten())}
            l_pts_list, r_pts_list, l_obj_list = [], [], []
            for cid in common_ids:
                l_pts_list.append(l_pts[l_id_to_idx[int(cid)]])
                r_pts_list.append(r_pts[r_id_to_idx[int(cid)]])
                l_obj_list.append(l_obj[l_id_to_idx[int(cid)]])

            l_pts_m = np.array(l_pts_list).reshape(-1, 1, 2)
            r_pts_m = np.array(r_pts_list).reshape(-1, 1, 2)
            l_obj_m = np.array(l_obj_list).reshape(-1, 1, 3)

            ok_l, rvec_l, tvec_l = cv2.solvePnP(
                l_obj_m, l_pts_m, left_K, left_dist, flags=cv2.SOLVEPNP_ITERATIVE)
            ok_r, rvec_r, tvec_r = cv2.solvePnP(
                l_obj_m, r_pts_m, right_K, right_dist, flags=cv2.SOLVEPNP_ITERATIVE)

            if not ok_l or not ok_r:
                cprint(f"[标定] PnP 失败，跳过", "red")
                continue

            # 板子原点(0,0,0)在左右相机中的坐标
            board_in_lcam = tvec_l.flatten()
            board_in_rcam = tvec_r.flatten()

            # 转到基座坐标系
            board_in_lbase = (T_lb_cam[:3, :3] @ board_in_lcam + T_lb_cam[:3, 3])
            board_in_rbase = (T_rb_cam[:3, :3] @ board_in_rcam + T_rb_cam[:3, 3])

            left_points_base.append(board_in_lbase)
            right_points_base.append(board_in_rbase)

            cprint(f"[标定] 检测成功  左:{l_n} 右:{r_n}  公共:{len(common_ids)} 角点", "green")
            cprint(f"  板子在左基座: {board_in_lbase*1000}", "green")
            cprint(f"  板子在右基座: {board_in_rbase*1000}", "green")

            if len(left_points_base) >= MIN_PAIRS:
                more = input(f"\n继续添加? (y/n, 建议 ≥{MIN_PAIRS}): ").strip().lower()
                if more != "y":
                    break
            else:
                cprint(f"  还需至少 {MIN_PAIRS - len(left_points_base)} 对", "yellow")

        if len(left_points_base) < MIN_PAIRS:
            cprint(f"[标定] 至少需要 {MIN_PAIRS} 对有效数据，当前 {len(left_points_base)}", "red")
            return False

        # ---- SVD 直接求解 T_right_to_left ----
        left_pts = np.array(left_points_base)   # Nx3, in left_base
        right_pts = np.array(right_points_base)  # Nx3, in right_base

        # 去中心化
        centroid_l = np.mean(left_pts, axis=0)
        centroid_r = np.mean(right_pts, axis=0)
        H = (right_pts - centroid_r).T @ (left_pts - centroid_l)

        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = centroid_l - R @ centroid_r

        T_right_to_left = np.eye(4)
        T_right_to_left[:3, :3] = R
        T_right_to_left[:3, 3] = t

        # 拟合误差
        left_pred = (R @ right_pts.T).T + t
        residuals = np.linalg.norm(left_pts - left_pred, axis=1) * 1000
        mean_res = np.mean(residuals)
        max_res = np.max(residuals)

        cprint(f"\n[标定] SVD 拟合 ({len(left_pts)} 位置):", "cyan")
        cprint(f"  残差: 平均 {mean_res:.1f}mm  最大 {max_res:.1f}mm", "yellow")
        for i, r in enumerate(residuals):
            outlier = r > mean_res * 2.5
            tag = "  <-- 离群!" if outlier else ""
            cprint(f"  位置 {i+1}: {r:.2f}mm{tag}", "red" if outlier else "green")

        # ---- 留一交叉验证 ----
        if len(left_pts) >= 4:
            loo_errors = []
            for i in range(len(left_pts)):
                mask = np.ones(len(left_pts), dtype=bool)
                mask[i] = False
                lp = left_pts[mask]; rp = right_pts[mask]
                cl = np.mean(lp, axis=0); cr = np.mean(rp, axis=0)
                H2 = (rp - cr).T @ (lp - cl)
                U2, _, Vt2 = np.linalg.svd(H2)
                R2 = Vt2.T @ U2.T
                if np.linalg.det(R2) < 0:
                    Vt2[-1, :] *= -1; R2 = Vt2.T @ U2.T
                t2 = cl - R2 @ cr
                err = np.linalg.norm(left_pts[i] - (R2 @ right_pts[i] + t2)) * 1000
                loo_errors.append(err)
            loo_mean = np.mean(loo_errors)
            loo_max = np.max(loo_errors)
            cprint(f"\n[标定] 留一交叉验证:", "cyan")
            for i, err in enumerate(loo_errors):
                outlier = err > loo_mean * 2.5
                tag = "  <-- 不可靠!" if outlier else ""
                cprint(f"  位置 {i+1}: {err:.2f}mm{tag}", "red" if outlier else "green")
            cprint(f"  平均: {loo_mean:.2f}mm  最大: {loo_max:.2f}mm", "yellow")
            if loo_max > 10:
                cprint(f"\n[标定] 交叉验证最大误差 {loo_max:.1f}mm > 10mm，标定可能不可靠", "yellow")
            else:
                cprint(f"\n[标定] 交叉验证通过 (最大 {loo_max:.1f}mm < 10mm)", "green")

        import tf.transformations as tr
        cprint(f"\n[标定] 最终变换 T_right_base → left_base:", "green")
        for row in T_right_to_left:
            cprint(f"  [{row[0]:8.5f} {row[1]:8.5f} {row[2]:8.5f} {row[3]:8.5f}]", "green")

        tx, ty, tz = T_right_to_left[:3, 3] * 1000
        euler = np.degrees(tr.euler_from_matrix(T_right_to_left, axes='sxyz'))
        cprint(f"\n  平移 (mm):  x={tx:.1f}  y={ty:.1f}  z={tz:.1f}", "green")
        cprint(f"  旋转 (deg): rx={euler[0]:.2f}  ry={euler[1]:.2f}  rz={euler[2]:.2f}", "green")

        # ---- 保存 ----
        save = input("\n保存到 robot_config.json? (y/n): ").strip().lower()
        if save == "y":
            save_calibration(T_right_to_left, len(left_pts),
                             error_mm=mean_res)
        else:
            cprint("[标定] 未保存", "yellow")

        return True

    finally:
        preview.stop()


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
    if "reprojection_error_px" in calib:
        cprint(f"  重投影误差: {calib['reprojection_error_px']}px", "cyan")
        cprint("  (旧版数据)", "yellow")
    if "mean_error_mm" in calib:
        cprint(f"  位置间偏差: {calib['mean_error_mm']}mm", "cyan")

    cprint("\n  T_right_to_left:", "cyan")
    for row in T:
        cprint(f"    [{row[0]:8.5f} {row[1]:8.5f} {row[2]:8.5f} {row[3]:8.5f}]", "cyan")

    cprint(f"\n  平移 (mm): x={T[0,3]*1000:.1f}  y={T[1,3]*1000:.1f}  z={T[2,3]*1000:.1f}", "cyan")

    import tf.transformations as tr
    euler = np.degrees(tr.euler_from_matrix(T, axes='sxyz'))
    cprint(f"  旋转 (deg): rx={euler[0]:.2f}  ry={euler[1]:.2f}  rz={euler[2]:.2f}", "cyan")


# ---------------------------------------------------------------------------
# 验证
# ---------------------------------------------------------------------------

def verify():
    calib = load_calibration()
    if calib is None:
        cprint("[标定] 尚未标定", "red")
        return

    T_r2l_stored = np.array(calib["T_right_to_left"])

    if not _init_ros("extract_stereo_calib_verify"):
        cprint("[标定] 需要 ROS 环境", "red")
        return

    board = create_charuco_board()
    left_K, left_dist = get_camera_intrinsics(CAMERA_SERIALS["left"])
    right_K, right_dist = get_camera_intrinsics(CAMERA_SERIALS["right"])

    preview = LivePreview(board, left_K, left_dist, right_K, right_dist)
    preview.start()

    try:
        cprint("\n=== 多点验证 (反算 T_right_to_left，不依赖臂位姿一致) ===", "cyan")
        cprint("[验证] 至少测试 3 个位置", "yellow")

        errors_mm = []
        round_num = 0

        while True:
            round_num += 1
            print("\n" + "-" * 50)
            cprint(f"[验证] 第 {round_num} 个位置", "cyan")
            input("放好 Charuco 板后按 Enter 拍照 ...")

            left_img, right_img = preview.grab_frames()
            if left_img is None or right_img is None:
                cprint("[验证] 相机帧未就绪", "red")
                continue

            l_pts, l_obj, l_ids, l_n = detect_charuco(board, left_img)
            r_pts, r_obj, r_ids, r_n = detect_charuco(board, right_img)

            if l_pts is None or r_pts is None:
                cprint(f"[验证] 检测失败 (左:{l_n} 右:{r_n})", "red")
                continue

            # 公共 id 精确匹配
            common_ids = np.intersect1d(l_ids.flatten(), r_ids.flatten())
            if len(common_ids) < 6:
                cprint(f"[验证] 公共角点仅 {len(common_ids)} 个", "red")
                continue

            l_id_to_idx = {int(id_): i for i, id_ in enumerate(l_ids.flatten())}
            r_id_to_idx = {int(id_): i for i, id_ in enumerate(r_ids.flatten())}
            l_pts_list, r_pts_list, l_obj_list = [], [], []
            for cid in common_ids:
                l_pts_list.append(l_pts[l_id_to_idx[int(cid)]])
                r_pts_list.append(r_pts[r_id_to_idx[int(cid)]])
                l_obj_list.append(l_obj[l_id_to_idx[int(cid)]])

            l_pts_m = np.array(l_pts_list).reshape(-1, 1, 2)
            r_pts_m = np.array(r_pts_list).reshape(-1, 1, 2)
            l_obj_m = np.array(l_obj_list).reshape(-1, 1, 3)

            ok_l, rvec_l, tvec_l = cv2.solvePnP(l_obj_m, l_pts_m, left_K, left_dist,
                                                  flags=cv2.SOLVEPNP_ITERATIVE)
            ok_r, rvec_r, tvec_r = cv2.solvePnP(l_obj_m, r_pts_m, right_K, right_dist,
                                                  flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok_l or not ok_r:
                cprint("[验证] PnP 失败", "red")
                continue

            R_l, _ = cv2.Rodrigues(rvec_l)
            T_board_lcam = np.eye(4); T_board_lcam[:3, :3] = R_l; T_board_lcam[:3, 3] = tvec_l.flatten()
            R_r, _ = cv2.Rodrigues(rvec_r)
            T_board_rcam = np.eye(4); T_board_rcam[:3, :3] = R_r; T_board_rcam[:3, 3] = tvec_r.flatten()

            T_rcam_to_lcam = T_board_lcam @ np.linalg.inv(T_board_rcam)

            try:
                T_lb_cam = lookup_transform(LEFT_BASE_FRAME, LEFT_CAM_FRAME)
                T_rb_cam = lookup_transform(RIGHT_BASE_FRAME, RIGHT_CAM_FRAME)
            except RuntimeError as e:
                cprint(f"[验证] TF 查询失败: {e}", "red")
                continue

            # 用当前 PnP + 当前 TF 反算 T_right_to_left（此值不应随臂位姿变化）
            T_r2l_current = T_lb_cam @ T_rcam_to_lcam @ np.linalg.inv(T_rb_cam)

            diff_t = np.linalg.norm(T_r2l_current[:3, 3] - T_r2l_stored[:3, 3]) * 1000
            errors_mm.append(diff_t)

            color = "green" if diff_t < 5 else ("yellow" if diff_t < 10 else "red")
            cprint(f"[验证] T_r2l 平移差: {diff_t:.2f}mm  "
                   f"(当前: {T_r2l_current[:3,3]*1000}  存储: {T_r2l_stored[:3,3]*1000})", color)

            if round_num >= 3:
                more = input("\n继续验证? (y/n): ").strip().lower()
                if more != "y":
                    break

        if not errors_mm:
            return

        mean_e = np.mean(errors_mm)
        max_e = np.max(errors_mm)
        cprint(f"\n[验证] {'─' * 40}", "cyan")
        cprint(f"  位置数: {len(errors_mm)}", "cyan")
        cprint(f"  T_r2l 平移差  平均: {mean_e:.2f}mm  最大: {max_e:.2f}mm", "yellow")

        if mean_e < 5:
            cprint("  结论: 精度良好 (<5mm)，可用于双臂协作", "green")
        elif mean_e < 10:
            cprint("  结论: 精度一般 (5-10mm)", "yellow")
        else:
            cprint("  结论: 精度不足 (>10mm)，建议重新标定", "red")

    finally:
        preview.stop()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="双臂立体标定工具 (Charuco 板 — 拍照模式，不依赖视频流)")
    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("calibrate", help="交互式拍照标定")
    subparsers.add_parser("show", help="查看标定结果")
    subparsers.add_parser("verify", help="多点验证标定精度")

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


if __name__ == "__main__":
    main()
