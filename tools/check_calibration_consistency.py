#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼外参多位姿一致性检验
========================
让臂依次走 grasp1-4 四个观察位, 每个位姿下用各自时刻的 T_base_to_cam 把参照物
(YOLO 检测框底边中心) 反算到 base 系。外参若准, 四个 base 坐标应重合在深度
噪声 (~2mm) 内; 若外参有旋转误差 δ, 不同位姿的估计互相矛盾 ≈ δ(rad) × 杠杆臂。

用框底边中心而不是框中心: 果子近似球, 不同视角下框中心对应球面不同点 (几 mm
的真实几何差), 底边始终对应果子与桌面接触的底缘, 物理点基本不动。

用法:
    conda activate anygrasp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.../nvidia/cudnn/lib
    python3 tools/check_calibration_consistency.py [物体名]   # 默认 orange,apple,fruit

结果解读:
    < 3mm   外参健康
    3-8mm   有偏差, 抓取边缘目标时会感到偏
    > 8mm   建议重标手眼外参 (mount_camera.py 的 T_matrix)
"""

import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from core.transforms import pixel_to_camera_point2
from skills.pick_and_place import PickAndPlaceSkill


def object_base_points(skill, class_names):
    """YOLO 检测所有框 -> [(base 点, 描述)]。跨位姿聚类后配对同一物体。"""
    rgb, depth = skill.get_camera_obs()
    skill.save_current_transformation()
    detections = skill.perception.detect_objects(
        rgb, class_names.split(","), conf=0.03)
    pts = []
    for det in detections:
        x1, y1, x2, y2 = [int(round(v)) for v in det[:4]]
        u = (x1 + x2) / 2.0
        v = max(0, y2 - 2)
        for half in (3, 6, 12, 24):
            patch = depth[max(0, v - half):v + half + 1,
                          max(0, int(u) - half):int(u) + half + 1]
            valid = patch[patch > 0]
            if len(valid) >= 5:
                break
        else:
            continue
        z_m = float(np.median(valid)) * 1e-3
        cam_pt = pixel_to_camera_point2(np.array([[u, v]], dtype=float), z_m)[0]
        base_pt = (np.asarray(skill.T_base_to_cam) @ np.append(cam_pt, 1.0))[:3]
        pts.append(base_pt)
    return pts


def main():
    class_names = sys.argv[1] if len(sys.argv) > 1 else "orange,apple,fruit"
    skill = PickAndPlaceSkill()
    all_pts = []      # [(pose_name, base_pt)]
    try:
        for key in ("grasp1", "grasp2", "grasp3", "grasp4"):
            if not skill.control_arm(pose_type=key, speed=30):
                print(f"[{key}] 到不了, 跳过")
                continue
            time.sleep(1.0)
            pts = object_base_points(skill, class_names)
            if not pts:
                print(f"[{key}] 没检出参照物, 跳过")
                continue
            for pt in pts:
                all_pts.append((key, pt))
                print(f"[{key}] 检测点 base 系: "
                      f"({pt[0]:+.4f}, {pt[1]:+.4f}, {pt[2]:+.4f}) m")
    finally:
        skill.control_arm(pose_type="grasp1", speed=30)

    if len({p for p, _ in all_pts}) < 2:
        print("[结果] 有效观测不足 2 个位姿, 无法检验")
        return 1

    # 跨位姿聚类 (40mm 阈值): 桌上有多个果子, 只有"每个位姿都看见的同一个
    # 物体"的簇才反映外参一致性。
    CLUSTER_M = 0.04
    best_cluster = []
    for i, (pi, pti) in enumerate(all_pts):
        cluster = [(pi, pti)]
        for pj, ptj in all_pts[i + 1:]:
            if pj in {p for p, _ in cluster}:
                continue
            if min(np.linalg.norm(ptj - pt) for _, pt in cluster) < CLUSTER_M:
                cluster.append((pj, ptj))
        if len({p for p, _ in cluster}) > len({p for p, _ in best_cluster}):
            best_cluster = cluster

    poses_unique = []
    for p, _ in best_cluster:
        if p not in poses_unique:
            poses_unique.append(p)
    if len(poses_unique) < 2:
        print(f"[结果] 没有一个物体被 2 个以上位姿共同看见 "
              f"(最大簇只覆盖 {poses_unique}), 参照物可能被挡/太远")
        return 1

    pts = np.array([pt for _, pt in best_cluster])
    center = pts.mean(axis=0)
    dists = np.linalg.norm(pts - center, axis=1)
    print(f"\n[结果] 同一物体的 {len(pts)} 个观测 ({', '.join(poses_unique)}), "
          f"聚类阈值 {CLUSTER_M*1000:.0f}mm")
    print(f"  均值: ({center[0]:+.4f}, {center[1]:+.4f}, {center[2]:+.4f}) m")
    print(f"  各观测相对均值的偏差 (mm): "
          + ", ".join(f"{p}:{d*1000:.1f}" for p, d in zip(poses_unique, dists)))
    print(f"  最大偏差: {dists.max()*1000:.1f} mm")
    if dists.max() * 1000 < 3:
        print("  结论: 外参一致性健康 (深度噪声量级)")
    elif dists.max() * 1000 < 8:
        print("  结论: 外参有偏差, 抓画面边缘目标时会感到偏, 建议重标")
    else:
        print("  结论: 外参明显不准, 强烈建议重标手眼外参 (mount_camera.py T_matrix)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
