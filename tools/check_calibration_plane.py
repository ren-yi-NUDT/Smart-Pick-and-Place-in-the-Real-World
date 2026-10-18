#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼外参桌面平面检验 (比检测框锚点更硬)
========================================
桌面是大平面且真实水平。每个观察位姿: 取深度有效点 -> 变换到 base 系 ->
RANSAC-ish 拟合最大平面 -> 报高度 z0 与法向。外参/内参若有旋转误差 δ,
不同位姿拟合的平面会绕不同轴倾斜 ~δ, 高度差也随位姿摆动。
不依赖物体检测, 几万点拟合, 稳健到 ~1-2mm。

用法: 同 check_calibration_consistency.py
"""

import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from skills.pick_and_place import PickAndPlaceSkill


def fit_table_plane(skill, skip=3):
    """当前帧深度点 -> base 系 -> RANSAC 取最大平面。返回 (z0, 法向, 内点数)。

    用 open3d segment_plane 而不是"取 z 最低 40% 点再收紧"的启发式: 视野里
    有杂物/玻璃时启发式会被带偏, 拟合出假平面 (2026-10-10 误报过 7.9°/14.6°)。
    """
    _, depth = skill.get_camera_obs()
    skill.save_current_transformation()
    T = np.asarray(skill.T_base_to_cam)
    H, W = depth.shape
    v, u = np.mgrid[0:H:skip, 0:W:skip]
    z = depth[::skip, ::skip].astype(float) * 1e-3
    m = z > 0.05
    u, v, z = u[m].astype(float), v[m].astype(float), z[m]
    if len(z) < 500:
        return None
    from core.transforms import _get_intrinsics
    fx, fy, cx, cy = _get_intrinsics("right")
    cam = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)
    base = cam @ T[:3, :3].T + T[:3, 3]

    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(base)
    plane, inliers = pcd.segment_plane(0.006, 3, 2000)
    a, b, c, d = plane
    if c < 0:
        a, b, c, d = -a, -b, -c, -d
    return d / c, np.array([a, b, c]), len(inliers)


def main():
    skill = PickAndPlaceSkill()
    rows = []
    try:
        for key in ("grasp1", "grasp2", "grasp3", "grasp4"):
            if not skill.control_arm(pose_type=key, speed=30):
                print(f"[{key}] 到不了, 跳过")
                continue
            time.sleep(1.0)
            r = fit_table_plane(skill)
            if r is None:
                print(f"[{key}] 桌面拟合失败")
                continue
            z0, n, npts = r
            rows.append((key, z0, n))
            tilt = np.degrees(np.arccos(np.clip(n[2], -1, 1)))
            print(f"[{key}] 桌面高度 z={z0:+.4f} m  内点={npts}  "
                  f"法向=({n[0]:+.4f},{n[1]:+.4f},{n[2]:+.4f})  倾角={tilt:.2f}°")
    finally:
        skill.control_arm(pose_type="grasp1", speed=30)

    if len(rows) < 2:
        print("[结果] 拟合成功的位姿不足, 无法检验")
        return 1
    zs = np.array([z for _, z, _ in rows])
    print(f"\n[结果] 桌面高度: 均值 {zs.mean()*1000:+.1f} mm, "
          f"散布 (max-min) {(zs.max()-zs.min())*1000:.1f} mm")
    ns = np.array([n for _, _, n in rows])
    pair = max(
        ((np.degrees(np.arccos(np.clip(ns[i] @ ns[j], -1, 1))), rows[i][0], rows[j][0])
         for i in range(len(ns)) for j in range(i + 1, len(ns))),
        default=(0.0, "", ""))
    print(f"  平面间最大夹角: {pair[0]:.2f}° ({pair[1]} vs {pair[2]})")
    if zs.ptp() * 1000 < 4 and pair[0] < 0.5:
        print("  结论: 外参一致性健康")
    elif zs.ptp() * 1000 < 12 and pair[0] < 2:
        print("  结论: 有轻微偏差, 可接受")
    else:
        print("  结论: 外参有系统偏差 (旋转误差 ~ 平面夹角), 建议重标")
    return 0


if __name__ == "__main__":
    sys.exit(main())
