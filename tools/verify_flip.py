#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZYZ 翻转的真机验证
==================
整张位姿表被换到了腕部有余量的等价解 (J4+180, -J5, J6+180)。这个变换在数学上保持末端
位姿不变, 但"数学上"和"真机上"是两回事 —— 关节限位、连杆干涉、线缆走向都只在真机上
才知道。这个工具用一个**独立于运动学模型**的仪器去量: 相机。

相机是 link_6 的固定子节点 (mount_camera.py: parent_frame='link_6'), 所以末端位姿一样,
画面就必须一模一样。于是:

    翻转位姿拍一张, 原始位姿拍一张, 比像素差。

判据不是"差多少算大", 而是**跟噪声底比**: 在同一个位姿上连拍两张, 得到的就是当前光照
下的噪声底。翻转带来的差异如果跟噪声底一个量级, 两个位姿就是同一个位姿。

每个位姿的走法 (中间回 home 而不是直接原地翻腕): 翻转 P -> home -> 原始 P。
原地翻腕只有 J4/J5/J6 变, 工具绕腕心划一道弧 —— 贴桌面的位姿 (desk_pose_3 静止时最低
0.032m) 有扫到桌面的风险。绕 home 多走两步, 换掉这个风险。

用法:
    conda activate anygrasp
    python3 tools/verify_flip.py --plan              # 只打印计划, 不动臂
    python3 tools/verify_flip.py --poses home        # 先验一个
    python3 tools/verify_flip.py                     # 默认那组
    python3 tools/verify_flip.py --poses home,grasp1 --speed 5
"""

import argparse
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from core.backends import create_arm, create_camera   # noqa: E402
from core.config import Config                        # noqa: E402

# 默认验这一组: 最常用的 (home/grasp1/place1) + 最贴桌面的 (desk_pose_3) + 递送/扔垃圾。
DEFAULT_POSES = ["home", "grasp1", "grasp2", "place1", "handover_pose",
                 "throw_to_trash_pose", "desk_pose_3"]

JOINTS = ["J%d" % (i + 1) for i in range(6)]


def raw_poses():
    with open(os.path.join(ROOT, "robot_config.json")) as f:
        cfg = json.load(f)
    out = {k: v for k, v in cfg.get("default_traj_js", {}).items() if "J1" in v}
    out.update({k: v for k, v in cfg.items()
                if isinstance(v, dict) and "J1" in v})
    return out


def as_list(pose):
    return [pose[j] for j in JOINTS]


def frame(cam):
    """连拍两张, 返回 (第一张, 第二张) —— 第二张用来量当前光照的噪声底。"""
    cam.get_rgbd()
    time.sleep(0.2)
    a, _ = cam.get_rgbd()
    time.sleep(0.2)
    b, _ = cam.get_rgbd()
    return np.asarray(a, dtype=np.int16), np.asarray(b, dtype=np.int16)


def diff(a, b):
    d = np.abs(a - b)
    return float(d.mean()), float(d.max())


def main():
    ap = argparse.ArgumentParser(description="用相机验证 ZYZ 翻转在真机上等价")
    ap.add_argument("--poses", default=",".join(DEFAULT_POSES),
                    help="要验的位姿, 逗号分隔 (默认: %s)" % ",".join(DEFAULT_POSES))
    ap.add_argument("--speed", type=int, default=10, help="关节速度百分比 (默认 10, 低速)")
    ap.add_argument("--settle", type=float, default=1.0, help="到位后等多久再拍 (秒)")
    ap.add_argument("--plan", action="store_true", help="只打印计划, 不动臂")
    args = ap.parse_args()

    cfg = Config()
    raw = raw_poses()
    names = [n.strip() for n in args.poses.split(",") if n.strip()]

    for n in names:
        if cfg.get_pose(n) is None or n not in raw:
            print("[error] 未知位姿: %s" % n)
            return 1

    print("=== 计划 (每个位姿 4 次运动, 速度 %d%%) ===" % args.speed)
    for n in names:
        f, r = cfg.get_pose(n), raw[n]
        moved = {j: abs(f[j] - r[j]) for j in JOINTS}
        print("  %-24s J4 %8.3f -> %8.3f   J5 %8.3f -> %8.3f   J6 %8.3f -> %8.3f"
              % (n, r["J4"], f["J4"], r["J5"], f["J5"], r["J6"], f["J6"]))
        print("  %-24s 翻转改变的关节: %s"
              % ("", ", ".join("%s %.2f°" % (j, moved[j]) for j in JOINTS if moved[j] > 1e-6)))
    print("  走法: home -> 翻转 P -> home -> 原始 P  (P = 每个位姿)")
    if args.plan:
        return 0

    arm = create_arm()
    if not arm.connect():
        print("[error] 连不上机械臂桥 (127.0.0.1:8010)")
        return 1
    cam = create_camera(save_path=os.path.join(ROOT, "log"))
    home = cfg.get_pose("home")

    print("\n=== 开始 ===")
    arm.move_to_named_pose(home, speed=args.speed)
    time.sleep(args.settle)

    rows = []
    for n in names:
        flip, rawp = cfg.get_pose(n), raw[n]

        arm.move_to_named_pose(flip, speed=args.speed)
        time.sleep(args.settle)
        a1, a2 = frame(cam)

        arm.move_to_named_pose(home, speed=args.speed)
        time.sleep(args.settle)
        arm.move_to_named_pose(rawp, speed=args.speed)
        time.sleep(args.settle)
        b1, _ = frame(cam)

        noise_mean, noise_max = diff(a1, a2)
        flip_mean, flip_max = diff(a1, b1)
        rows.append((n, noise_mean, flip_mean, flip_max))

        verdict = "同一姿态" if flip_mean <= max(noise_mean * 3.0, 1.5) else "**不一样**"
        print("  %-24s 噪声底 %.2f (max %.0f)   翻转差 %.2f (max %.0f)   %s"
              % (n, noise_mean, noise_max, flip_mean, flip_max, verdict))

        for tag, img in (("flip", a1), ("raw", b1)):
            import cv2
            p = os.path.join(ROOT, "log", "verify_flip_%s_%s.png" % (n, tag))
            cv2.imwrite(p, img.astype(np.uint8))

        arm.move_to_named_pose(home, speed=args.speed)
        time.sleep(args.settle)

    print("\n=== 汇总 ===")
    print("  %-24s %10s %10s" % ("位姿", "噪声底", "翻转差"))
    for n, nm, fm, _ in rows:
        print("  %-24s %10.2f %10.2f" % (n, nm, fm))
    bad = [n for n, nm, fm, _ in rows if fm > max(nm * 3.0, 1.5)]
    print("\n  不通过的: %s" % (bad if bad else "无"))
    print("  图存到 log/verify_flip_<位姿>_{flip,raw}.png, 可以直接对着看")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
