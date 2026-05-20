#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位姿录制工具
============
开发阶段使用的独立工具脚本，用于录制机械臂位姿并保存到 JSON 文件。

直接连接机械臂硬件（Robotic_Arm SDK）读取当前关节状态。

录制结果保存在 recorded_poses.json，需手动复制到 robot_config.json 对应的
left/right arm 配置中才会被 skill 流水线使用。

机械臂映射:
    left  → 192.168.1.19 (灵巧手)
    right → 192.168.1.18 (夹爪)

使用方式:

    # 录制单个位姿（必须指定 --arm）
    python3 tools/pose_record.py record --name grasp1 --arm left
    python3 tools/pose_record.py record --name handover_pose --arm right --desc "交接位"

    # 交互式连续录制
    python3 tools/pose_record.py record -i --arm left
    python3 tools/pose_record.py record -i --arm right

    # 列出所有已录制的位姿
    python3 tools/pose_record.py list

    # 删除位姿
    python3 tools/pose_record.py delete --name grasp1

操作流程:
    1. 开启示教模式，手动将机械臂拖到目标位姿
    2. 运行录制命令，当前关节角度自动保存到 recorded_poses.json
    3. 将录制的 joint_angles_deg 转为 {"J1": ..., "J7": ...} 格式
       复制到 robot_config.json 对应 arm 的 default_traj_js 或顶层字段
"""

import os
import sys
import json
import argparse
from datetime import datetime
from termcolor import cprint

# 机械臂接口 -- 直接连接硬件以读取当前关节状态
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POSE_DIR = os.path.join(PROJECT_ROOT, "recorded_poses")

POSE_FILES = {
    "left": os.path.join(POSE_DIR, "left.json"),
    "right": os.path.join(POSE_DIR, "right.json"),
}

ARM_CONFIGS = {
    "left": {"ip": "192.168.1.19", "label": "左臂（灵巧手）"},
    "right": {"ip": "192.168.1.18", "label": "右臂（夹爪）"},
}
ARM_PORT = 8080


# ---------------------------------------------------------------------------
# 录制功能
# ---------------------------------------------------------------------------

def _load_poses(pose_file):
    """加载已保存的位姿"""
    if os.path.exists(pose_file):
        with open(pose_file, "r") as f:
            return json.load(f)
    return {}


def _save_poses(poses, pose_file):
    """保存位姿到文件"""
    with open(pose_file, "w") as f:
        json.dump(poses, f, indent=2)
    cprint(f"[pose_record] 位姿已保存到 {pose_file}", "green")


def _print_pose(pose_data):
    """打印位姿信息"""
    print("=" * 50)
    print(f"名称: {pose_data['name']}")
    print(f"描述: {pose_data.get('description', 'N/A')}")
    print(f"时间: {pose_data['timestamp']}")

    joints = pose_data["joint_angles_deg"]
    print("\n关节角度 (度):")
    for i, j in enumerate(joints):
        print(f"  J{i+1}: {j:.3f} deg")

    pose = pose_data.get("end_pose")
    if pose is not None:
        print("\n末端位姿:")
        print(f"  位置: x={pose[0]:.4f}, y={pose[1]:.4f}, z={pose[2]:.4f} (米)")
        print(f"  姿态: rx={pose[3]:.4f}, ry={pose[4]:.4f}, rz={pose[5]:.4f} (弧度)")
    print("=" * 50)


def record_pose(name, description="", arm_side="left"):
    """
    录制当前机械臂位姿

    Args:
        name: 位姿名称
        description: 位姿描述
        arm_side: "left" 或 "right"

    Returns:
        bool: 是否录制成功
    """
    os.makedirs(POSE_DIR, exist_ok=True)
    pose_file = POSE_FILES[arm_side]
    arm_cfg = ARM_CONFIGS[arm_side]
    arm_ip = arm_cfg["ip"]

    # 连接机械臂硬件
    try:
        arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
        arm_handle = arm.rm_create_robot_arm(arm_ip, ARM_PORT, level=3)
        if arm_handle.id == -1:
            cprint(f"[pose_record] 无法连接{arm_cfg['label']} {arm_ip}:{ARM_PORT}", "red")
            return False
        cprint(f"[pose_record] 已连接{arm_cfg['label']}, ID: {arm_handle.id}", "green")
    except Exception as e:
        cprint(f"[pose_record] 连接机械臂失败: {e}", "red")
        return False

    try:
        # 读取当前关节状态
        tag, arm_state = arm.rm_get_current_arm_state()
        if tag != 0:
            cprint("[pose_record] 无法获取机械臂状态，尝试 rm_get_joint_degree ...", "yellow")
            tag2, joints = arm.rm_get_joint_degree()
            if tag2 != 0:
                cprint("[pose_record] rm_get_joint_degree 也失败", "red")
                return False
            pose_data = {
                "name": name,
                "description": description,
                "arm": arm_side,
                "timestamp": datetime.now().isoformat(),
                "joint_angles_deg": joints,
                "end_pose": None,
            }
        else:
            pose_data = {
                "name": name,
                "description": description,
                "arm": arm_side,
                "timestamp": datetime.now().isoformat(),
                "joint_angles_deg": arm_state["joint"],
                "end_pose": arm_state["pose"],
            }

        # 保存
        poses = _load_poses(pose_file)
        poses[name] = pose_data
        _save_poses(poses, pose_file)

        cprint(f"\n[pose_record] 已录制位姿: {name}", "cyan")
        _print_pose(pose_data)
        return True
    finally:
        # 注意: 保持连接不断开 (与原始行为一致)
        pass


def interactive_record(arm_side="left"):
    """交互式录制模式"""
    os.makedirs(POSE_DIR, exist_ok=True)
    pose_file = POSE_FILES[arm_side]
    arm_cfg = ARM_CONFIGS[arm_side]

    cprint(f"\n=== 交互式位姿录制模式 ({arm_cfg['label']}) ===", "cyan")
    cprint("输入位姿名称开始录制，输入 'q' 退出", "yellow")

    while True:
        print("\n" + "-" * 40)
        name = input("请输入位姿名称: ").strip()

        if name.lower() == "q":
            break

        if not name:
            cprint("名称不能为空", "red")
            continue

        desc = input("请输入描述（可选）: ").strip()
        record_pose(name, desc, arm_side=arm_side)

    cprint("\n[pose_record] 退出交互式录制模式", "yellow")


def list_poses(arm_side=None):
    """列出已录制的位姿"""
    if arm_side:
        sides = [arm_side]
    else:
        sides = ["left", "right"]

    for side in sides:
        pose_file = POSE_FILES[side]
        label = ARM_CONFIGS[side]["label"]
        poses = _load_poses(pose_file)

        if not poses:
            cprint(f"\n{label}: 没有已录制的位姿", "yellow")
            continue

        cprint(f"\n{label} ({len(poses)} 个):", "cyan")
        print("=" * 60)
        for name, data in poses.items():
            desc = data.get("description", "N/A")
            timestamp = data.get("timestamp", "N/A")
            print(f"  {name}: {desc} ({timestamp})")
        print("=" * 60)


def delete_pose(name, arm_side):
    """删除指定位姿"""
    pose_file = POSE_FILES[arm_side]
    poses = _load_poses(pose_file)

    if name not in poses:
        cprint(f"[pose_record] {ARM_CONFIGS[arm_side]['label']}中未找到位姿: {name}", "red")
        return False

    del poses[name]
    _save_poses(poses, pose_file)
    cprint(f"[pose_record] 已从{ARM_CONFIGS[arm_side]['label']}删除位姿: {name}", "green")
    return True


# ---------------------------------------------------------------------------
# CLI 入口
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="位姿录制工具 (开发阶段使用)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
机械臂映射:
  left  → 192.168.1.19 (灵巧手)
  right → 192.168.1.18 (夹爪)

操作流程:
  1. 开启示教模式，手动将机械臂拖到目标位姿
  2. 运行录制命令，当前关节角度自动保存到 recorded_poses.json
  3. 将录制的位姿复制到 robot_config.json 对应 arm 的配置中

示例:
  python3 tools/pose_record.py record --name grasp1 --arm left
  python3 tools/pose_record.py record --name handover_pose --arm right --desc "交接位"
  python3 tools/pose_record.py record -i --arm left
  python3 tools/pose_record.py record -i --arm right
  python3 tools/pose_record.py list
  python3 tools/pose_record.py delete --name grasp1
        """,
    )

    subparsers = parser.add_subparsers(dest="command", help="命令")

    # record 命令
    record_parser = subparsers.add_parser("record", help="录制位姿")
    record_parser.add_argument("--name", "-n", help="位姿名称")
    record_parser.add_argument("--desc", "-d", default="", help="位姿描述")
    record_parser.add_argument(
        "--interactive", "-i", action="store_true", help="交互式录制模式"
    )
    record_parser.add_argument(
        "--arm", "-a", required=True, choices=["left", "right"],
        help="选择机械臂: left=左臂(192.168.1.19), right=右臂(192.168.1.18)"
    )

    # list 命令
    list_parser = subparsers.add_parser("list", help="列出已录制的位姿")
    list_parser.add_argument(
        "--arm", "-a", choices=["left", "right"],
        help="筛选指定臂（不指定则显示全部）"
    )

    # delete 命令
    del_parser = subparsers.add_parser("delete", help="删除指定位姿")
    del_parser.add_argument("--name", "-n", required=True, help="位姿名称")
    del_parser.add_argument(
        "--arm", "-a", required=True, choices=["left", "right"],
        help="从哪条臂删除"
    )

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    if args.command == "record":
        if args.interactive:
            interactive_record(arm_side=args.arm)
        elif args.name:
            record_pose(args.name, args.desc, arm_side=args.arm)
        else:
            cprint("错误: 请指定 --name 或使用 --interactive 模式", "red")

    elif args.command == "list":
        list_poses(arm_side=args.arm if hasattr(args, 'arm') and args.arm else None)

    elif args.command == "delete":
        delete_pose(args.name, arm_side=args.arm)


if __name__ == "__main__":
    main()
