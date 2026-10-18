#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线路径碰撞检查
================
位姿之间的转移走的是裸关节插值 (rm_movej / arm bridge 的 js 命令), 既不做
Z 高度检查也不做碰撞检查 —— Twin 的 trajectory_generation2 才有 Z 安全检查,
而它只用在抓取/放置的笛卡尔轨迹上。

这个工具在 PyBullet 里把每一对位姿之间的关节插值走一遍, 报出末端/夹爪的最低
Z, 用来在真机运动之前发现"会扫到桌面"的路径。纯仿真, 不连硬件。

用法:
    conda activate anygrasp
    python3 tools/check_pose_paths.py                # 全部位姿两两检查
    python3 tools/check_pose_paths.py --table-z 0.10 # 低于该高度标红 (默认 0.10, 同 Twin)
    python3 tools/check_pose_paths.py --from home --to grasp1   # 只查一对
"""

import argparse
import itertools
import json
import os

import pybullet as p
import pybullet_data

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(ROOT, "robot_config.json")
URDF_PATH = os.path.join(
    ROOT, "dependence/smart_pick_and_place_ws/src/rm_description/urdf/SingleArm/rm65.urdf"
)

TOP_LEVEL_POSES = [
    "get_ready_to_handover_1st", "get_ready_to_handover_2nd", "handover_pose",
    "throw_to_trash_pose", "look_over_what_in_user_hand_pose",
    "desk_pose_1", "desk_pose_2", "desk_pose_3",
]

ARM_JOINTS = 6


def load_poses():
    with open(CONFIG_PATH) as f:
        config = json.load(f)
    poses = {k: v for k, v in config["default_traj_js"].items() if "J1" in v}
    poses.update({k: config[k] for k in TOP_LEVEL_POSES if k in config and "J1" in config[k]})
    return {name: [pose["J%d" % (i + 1)] for i in range(ARM_JOINTS)] for name, pose in poses.items()}


def build_robot():
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    uid = p.loadURDF(URDF_PATH, useFixedBase=True)
    return uid


def path_min_z(uid, start, end, steps):
    """关节线性插值走一遍, 返回 (最低Z, 最低Z时的link索引, 最大单关节变化量)"""
    lowest, lowest_link = 1e9, None
    for s in range(steps + 1):
        t = s / steps
        for j in range(ARM_JOINTS):
            angle = start[j] + (end[j] - start[j]) * t
            p.resetJointState(uid, j, angle * 3.141592653589793 / 180.0)
        for link in range(p.getNumJoints(uid)):
            z = p.getAABB(uid, link)[0][2]
            if z < lowest:
                lowest, lowest_link = z, link
    excursion = max(abs(a - b) for a, b in zip(start, end))
    return lowest, lowest_link, excursion


def main():
    ap = argparse.ArgumentParser(description="位姿间转移路径的离线 Z 检查")
    ap.add_argument("--table-z", type=float, default=0.10,
                    help="低于该高度视为会撞桌面 (米, 默认 0.10, 同 Twin 的 z_min_threshold)")
    ap.add_argument("--steps", type=int, default=60, help="每段插值步数 (默认 60)")
    ap.add_argument("--from", dest="src", help="只查这一对: 起始位姿名")
    ap.add_argument("--to", dest="dst", help="只查这一对: 目标位姿名")
    args = ap.parse_args()

    poses = load_poses()
    uid = build_robot()

    if args.src and args.dst:
        pairs = [(args.src, args.dst)]
    else:
        pairs = [(a, b) for a, b in itertools.permutations(poses, 2)]

    rows = []
    for a, b in pairs:
        if a not in poses:
            print("[error] 未知位姿名: %s" % a)
            return 1
        if b not in poses:
            print("[error] 未知位姿名: %s" % b)
            return 1
        low, link, exc = path_min_z(uid, poses[a], poses[b], args.steps)
        rows.append((low, a, b, link, exc))

    print("\n=== 单个位姿静止时的最低点 ===")
    print("%-34s %9s %8s %s" % ("位姿", "最低Z(m)", "最大关节差", "最低点link"))
    print("-" * 68)
    for name, joints in sorted(poses.items(),
                               key=lambda kv: path_min_z(uid, kv[1], kv[1], 1)[0]):
        low, link, _ = path_min_z(uid, joints, joints, 1)
        flag = "  <== 已低于桌面" if low < args.table_z else ""
        print("%-34s %9.3f %8s %s%s" % (name, low, "-", link, flag))

    rows.sort()
    print("\n=== 位姿间转移路径 ===")
    print("%-22s -> %-22s %9s %9s %s" % ("起始", "目标", "最低Z(m)", "最大关节差", "最低点link"))
    print("-" * 78)
    for low, a, b, link, exc in rows:
        flag = "  <== 撞桌面" if low < args.table_z else ""
        print("%-22s -> %-22s %9.3f %9.1f %s%s" % (a, b, low, exc, link, flag))

    bad = [r for r in rows if r[0] < args.table_z]
    print("\n共 %d 对, %d 对最低点低于 %.3f m" % (len(rows), len(bad), args.table_z))
    if bad:
        print("最危险: %s -> %s  最低 %.3f m" % (bad[0][1], bad[0][2], bad[0][0]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
