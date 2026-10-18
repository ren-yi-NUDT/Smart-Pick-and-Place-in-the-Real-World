#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼外参重标 (eye-in-hand, 棋盘格 AX=XB)
==========================================
前提: 标定板 (9x6 内角点, tools/make_chessboard_pdf.py 生成) 平放在臂前方桌面,
扫描期间不要动它。臂依次走到各观测位, 每处拍一帧:
  - findChessboardCorners + solvePnP 得 T_cam<-board (target2cam)
  - TF 查 T_base<-link_6 (gripper2base)
然后 cv2.calibrateHandEye 解 X = T_link6<-cam —— 即 mount_camera.py 里的
T_matrix (parent=link_6, child=cam_link_grasp, ROS 语义就是 T_parent<-child)。

用法:
    conda activate anygrasp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.../nvidia/cudnn/lib
    python3 tools/handeye_calibrate.py [--square-mm 25.0] [--poses grasp1,grasp2,grasp3,grasp4,place1,place2]

结果解读: 用解出的 X 反算各位姿的 T_base<-board, 平移散布 <5mm 可用。
输出写 tools/handeye_result.json, 矩阵贴进 mount_camera.py 的 T_matrix 后
重启 ROS bringup 生效。
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from core.transforms import _get_intrinsics
from skills.pick_and_place import PickAndPlaceSkill

PATTERN = (9, 6)  # 内角点: 10x7 方格


def solve_target2cam(rgb, square_m):
    """RGB -> T_cam<-board (4x4), 棋盘不完整/未检出返回 None。"""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    ok, corners = cv2.findChessboardCorners(
        gray, PATTERN,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
    objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * square_m
    fx, fy, cx, cy = _get_intrinsics("right")
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, None)
    if not ok:
        return None
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T


def residuals(T_base_board_list):
    """反算 T_base<-board 之间的最大平移差 (mm) 和最大旋转差 (deg)。"""
    ts = np.array([m[:3, 3] for m in T_base_board_list])
    t_max = max(np.linalg.norm(a - b)
                for i, a in enumerate(ts) for b in ts[i + 1:])
    rs = [m[:3, :3] for m in T_base_board_list]
    r_max = max(np.degrees(np.arccos(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))
                for i, a in enumerate(rs) for b in rs[i + 1:])
    return t_max * 1000, r_max


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--poses", default="grasp1,grasp2,grasp3,grasp4,place1,place2")
    args = ap.parse_args()

    skill = PickAndPlaceSkill()
    base, link6 = skill.config.base_link_name, skill.config.arm_end_link_name
    g2b, t2c = [], []  # gripper2base, target2cam
    used = []

    try:
        for name in [p.strip() for p in args.poses.split(",") if p.strip()]:
            if not skill.control_arm(pose_type=name, speed=30):
                print(f"[{name}] 到不了, 跳过")
                continue
            time.sleep(1.0)
            rgb, _ = skill.get_camera_obs()
            T_base_link6 = skill.transforms.get_transform_from_frame_to_frame(
                base, link6)[0]
            T_cam_board = solve_target2cam(rgb, args.square_mm / 1000.0)
            if T_cam_board is None:
                print(f"[{name}] 棋盘格没检出, 跳过")
                continue
            g2b.append(T_base_link6)
            t2c.append(T_cam_board)
            used.append(name)
            print(f"[{name}] ok  board@base 平移="
                  f"({(T_base_link6 @ T_cam_board)[:3, 3]})")
    finally:
        skill.control_arm(pose_type=used[0] if used else "grasp1", speed=30)

    if len(used) < 3:
        print(f"[结果] 有效位姿只有 {len(used)} 个 (<3), 至少要 3 个能看到棋盘的位姿")
        return 1

    best = None
    for method, label in ((cv2.CALIB_HAND_EYE_TSAI, "TSAI"),
                          (cv2.CALIB_HAND_EYE_PARK, "PARK")):
        R, t = cv2.calibrateHandEye(
            [m[:3, :3] for m in g2b], [m[:3, 3] for m in g2b],
            [m[:3, :3] for m in t2c], [m[:3, 3] for m in t2c], method=method)
        X = np.eye(4)
        X[:3, :3], X[:3, 3] = R, t.ravel()
        tb = [g @ X @ c for g, c in zip(g2b, t2c)]
        t_mm, r_deg = residuals(tb)
        print(f"\n[{label}]  反算 T_base<-board 散布: 平移 {t_mm:.1f} mm, 旋转 {r_deg:.2f}°")
        for name, m in zip(used, tb):
            print(f"  [{name}] board@base = ({m[0, 3]:+.4f}, {m[1, 3]:+.4f}, {m[2, 3]:+.4f})")
        if best is None or t_mm < best[1]:
            best = (X, t_mm, r_deg, label)

    X, t_mm, r_deg, label = best
    print(f"\n[结果] 取 {label}: 平移散布 {t_mm:.1f} mm, 旋转散布 {r_deg:.2f}°"
          f"  ({'可用' if t_mm < 5 else '偏大, 建议多加几个位姿重扫'})")
    print("X = T_link6<-cam (贴进 mount_camera.py 的 T_matrix):")
    print(np.array2string(X, separator=", ", precision=12))

    out = os.path.join(ROOT, "tools", "handeye_result.json")
    with open(out, "w") as f:
        json.dump({"T_link6_cam": X.tolist(), "residual_mm": t_mm,
                   "residual_deg": r_deg, "method": label, "poses": used},
                  f, indent=2)
    print(f"已保存: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
