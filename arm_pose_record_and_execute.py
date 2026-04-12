#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位姿录制与执行脚本
==================
支持录制机械臂位姿，并通过JSON执行预设动作序列。

使用方式:
    # 录制模式 - 录制当前位置并保存
    python3 pose_recorder.py record --name "home"

    # 执行模式 - 执行预设位姿
    python3 pose_recorder.py play --name "home"

    # 列出所有已录制的位姿
    python3 pose_recorder.py list

    # 执行动作序列（从JSON文件）
    python3 pose_recorder.py sequence --file sequence.json

    # 执行动作序列（从stdin）
    echo '{"sequence": [{"arm": "home", "hand": "open"}]}' | python3 pose_recorder.py sequence

动作序列文件格式 (JSON):
    {
        "sequence": [
            {"arm": "home", "hand": "open", "delay": 0.5},
            {"arm": "grasp1", "hand": "open"},
            {"hand": "close", "delay": 0.3},
            {"arm": "place1", "hand": [0,0,1000,1000,0,0]},
            {"hand": {"thumb": 1000, "index": 0}, "delay": 0.5}
        ]
    }

手势格式支持:
    - 预设名称: "open", "close", "peace", "rock", "pointing", "thumbs_up", "ok", "grab"
    - 数组: [小指, 无名指, 中指, 食指, 拇指, 拇指外展] (0=弯曲, 1000=张开)
    - 字典: {"pinky": 0, "ring": 0, "middle": 1000, "index": 1000, "thumb": 0, "thumb_abduct": 0}
"""

import os
import sys
import json
import socket
import struct
import argparse
import time
from datetime import datetime
from termcolor import cprint

# 机械臂接口
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


class PoseRecorder:
    """位姿录制与执行器"""

    # 默认配置
    DEFAULT_POSE_FILE = "./recorded_poses.json"
    ARM_IP = "192.168.1.19"
    ARM_PORT = 8080
    ARM_SERVER_HOST = "127.0.0.1"
    ARM_SERVER_PORT = 8010
    HAND_SERVER_HOST = "127.0.0.1"
    HAND_SERVER_PORT = 8000

    # 手势预设 (6值: [小指, 无名指, 中指, 食指, 拇指, 拇指外展])
    # 0=弯曲, 1000=张开
    HAND_GESTURES = {
        "open": [1000, 1000, 1000, 1000, 1000, 500],      # 全张开
        "close": [0, 0, 0, 0, 0, 0],                       # 握拳
        "peace": [0, 0, 1000, 1000, 0, 0],                 # 比耶
        "rock": [1000, 0, 0, 1000, 0, 0],                  # Rock手势
        "pointing": [0, 0, 0, 1000, 0, 0],                 # 指向 (食指)
        "thumbs_up": [0, 0, 0, 0, 1000, 800],              # 竖大拇指
        "ok": [800, 800, 800, 150, 150, 400],              # OK手势 (食指+拇指圈起，其他张开)
        "grab": [50, 50, 50, 100, 100, 0],                 # 抓取姿态
    }

    def __init__(self, pose_file=None):
        """
        初始化位姿录制器

        Args:
            pose_file: 位姿存储文件路径
        """
        self.pose_file = pose_file or self.DEFAULT_POSE_FILE
        self.arm = None
        self.arm_handle = None
        self.arm_server_client = None
        self.hand_server_client = None
        self.poses = self._load_poses()

    def _load_poses(self):
        """加载已保存的位姿"""
        if os.path.exists(self.pose_file):
            with open(self.pose_file, "r") as f:
                return json.load(f)
        return {}

    def _save_poses(self):
        """保存位姿到文件"""
        with open(self.pose_file, "w") as f:
            json.dump(self.poses, f, indent=2)
        cprint(f"[PoseRecorder] 位姿已保存到 {self.pose_file}", "green")

    # ==================== 机械臂连接 ====================

    def connect_arm(self):
        """连接机械臂（用于读取位姿）"""
        try:
            self.arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
            self.arm_handle = self.arm.rm_create_robot_arm(self.ARM_IP, self.ARM_PORT, level=3)
            if self.arm_handle.id == -1:
                cprint(f"[PoseRecorder] 无法连接机械臂 {self.ARM_IP}:{self.ARM_PORT}", "red")
                return False
            cprint(f"[PoseRecorder] 已连接机械臂, ID: {self.arm_handle.id}", "green")
            return True
        except Exception as e:
            cprint(f"[PoseRecorder] 连接机械臂失败: {e}", "red")
            return False

    def disconnect_arm(self):
        """断开机械臂连接（已禁用）"""
        # 已禁用，保持连接
        # if self.arm:
        #     self.arm.rm_delete_robot_arm()
        #     self.arm = None
        #     self.arm_handle = None
        #     cprint("[PoseRecorder] 已断开机械臂连接", "yellow")
        pass

    def connect_arm_server(self):
        """连接机械臂控制服务（用于执行位姿）"""
        try:
            self.arm_server_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.arm_server_client.connect((self.ARM_SERVER_HOST, self.ARM_SERVER_PORT))
            cprint(f"[PoseRecorder] 已连接机械臂服务 {self.ARM_SERVER_HOST}:{self.ARM_SERVER_PORT}", "green")
            return True
        except Exception as e:
            cprint(f"[PoseRecorder] 连接机械臂服务失败: {e}", "red")
            return False

    def disconnect_arm_server(self):
        """断开机械臂控制服务连接（已禁用）"""
        # 已禁用，保持连接
        # if self.arm_server_client:
        #     self.arm_server_client.close()
        #     self.arm_server_client = None
        #     cprint("[PoseRecorder] 已断开机械臂服务连接", "yellow")
        pass

    # ==================== 灵巧手控制 ====================

    def connect_hand_server(self):
        """连接灵巧手控制服务"""
        try:
            self.hand_server_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.hand_server_client.connect((self.HAND_SERVER_HOST, self.HAND_SERVER_PORT))
            cprint(f"[PoseRecorder] 已连接灵巧手服务 {self.HAND_SERVER_HOST}:{self.HAND_SERVER_PORT}", "green")
            return True
        except Exception as e:
            cprint(f"[PoseRecorder] 连接灵巧手服务失败: {e}", "red")
            return False

    def disconnect_hand_server(self):
        """断开灵巧手控制服务连接（已禁用）"""
        # 已禁用，保持连接
        # if self.hand_server_client:
        #     self.hand_server_client.close()
        #     self.hand_server_client = None
        #     cprint("[PoseRecorder] 已断开灵巧手服务连接", "yellow")
        pass

    def _parse_hand_gesture(self, hand_input):
        """
        解析手势输入，返回 6 值数组

        支持格式:
        - 预设名称: "open", "close", "peace", "rock", "pointing", "thumbs_up", "ok", "grab"
        - 数组: [0, 0, 1000, 1000, 0, 0]
        - 字典: {"pinky": 0, "ring": 0, "middle": 1000, "index": 1000, "thumb": 0, "thumb_abduct": 0}
        """
        if isinstance(hand_input, str):
            # 预设名称
            if hand_input in self.HAND_GESTURES:
                return self.HAND_GESTURES[hand_input]
            else:
                cprint(f"[PoseRecorder] 未知手势预设: {hand_input}", "red")
                return None
        elif isinstance(hand_input, list):
            # 直接数组
            if len(hand_input) == 6:
                return hand_input
            else:
                cprint(f"[PoseRecorder] 手指数组长度必须为6，收到: {len(hand_input)}", "red")
                return None
        elif isinstance(hand_input, dict):
            # 字典格式转数组
            finger_map = ["pinky", "ring", "middle", "index", "thumb", "thumb_abduct"]
            result = []
            for i, finger in enumerate(finger_map):
                result.append(hand_input.get(finger, 0))
            return result
        else:
            cprint(f"[PoseRecorder] 不支持的手势格式: {type(hand_input)}", "red")
            return None

    def play_hand_gesture(self, hand_input):
        """
        执行手势

        Args:
            hand_input: 手势输入（预设名/数组/字典）
        """
        gesture = self._parse_hand_gesture(hand_input)
        if gesture is None:
            return False

        if not self.hand_server_client:
            if not self.connect_hand_server():
                return False

        try:
            cmd = {
                "src": "/left_hand/movement_control",
                "type": "set",
                "cmd": gesture
            }
            msg = json.dumps(cmd).encode('utf-8')
            self.hand_server_client.sendall(msg)

            resp = json.loads(self.hand_server_client.recv(1024).decode('utf-8'))
            cprint(f"[PoseRecorder] 执行手势 {hand_input if isinstance(hand_input, str) else gesture}: {resp}", "green")
            return resp.get("value", False)
        except Exception as e:
            cprint(f"[PoseRecorder] 执行手势失败: {e}", "red")
            return False

    # ==================== 位姿录制 ====================

    def record_pose(self, name, description=""):
        """
        录制当前机械臂位姿

        Args:
            name: 位姿名称
            description: 位姿描述
        """
        if not self.arm:
            if not self.connect_arm():
                return False

        tag, arm_state = self.arm.rm_get_current_arm_state()
        if tag != 0:
            cprint("[PoseRecorder] 无法获取机械臂状态", "red")
            return False

        pose_data = {
            "name": name,
            "description": description,
            "timestamp": datetime.now().isoformat(),
            "joint_angles_deg": arm_state["joint"],
            "end_pose": arm_state["pose"]
        }

        self.poses[name] = pose_data
        self._save_poses()

        cprint(f"\n[PoseRecorder] 已录制位姿: {name}", "cyan")
        self._print_pose(pose_data)
        return True

    def _print_pose(self, pose_data):
        """打印位姿信息"""
        print("=" * 50)
        print(f"名称: {pose_data['name']}")
        print(f"描述: {pose_data.get('description', 'N/A')}")
        print(f"时间: {pose_data['timestamp']}")

        joints = pose_data["joint_angles_deg"]
        print("\n关节角度 (度):")
        for i, j in enumerate(joints):
            print(f"  J{i+1}: {j:.3f}°")

        pose = pose_data["end_pose"]
        print("\n末端位姿:")
        print(f"  位置: x={pose[0]:.4f}, y={pose[1]:.4f}, z={pose[2]:.4f} (米)")
        print(f"  姿态: rx={pose[3]:.4f}, ry={pose[4]:.4f}, rz={pose[5]:.4f} (弧度)")
        print("=" * 50)

    # ==================== 位姿执行 ====================

    def play_pose(self, name, speed=30, block=True):
        """
        执行预设位姿

        Args:
            name: 位姿名称
            speed: 移动速度 (0-100)
            block: 是否阻塞等待完成
        """
        if name not in self.poses:
            cprint(f"[PoseRecorder] 未找到位姿: {name}", "red")
            return False

        pose_data = self.poses[name]
        joint_angles = pose_data["joint_angles_deg"]

        if not self.arm_server_client:
            if not self.connect_arm_server():
                return False

        try:
            # 将关节角度数组转换为 {J1, J2, ...} 格式
            joint_dict = {f"J{i+1}": j for i, j in enumerate(joint_angles)}

            # 使用 ArmController 协议发送指令
            cmd = {
                "srv": "/right_arm/movement_control",
                "cmd": [
                    {"type": "start", "act": []},
                    {"type": "js", "act": joint_dict, "speed": speed, "block": block},
                    {"type": "end", "act": []}
                ]
            }
            msg = json.dumps(cmd).encode('utf-8')

            # 发送 4 字节长度前缀 + JSON 数据
            length_prefix = struct.pack('>I', len(msg))
            self.arm_server_client.sendall(length_prefix + msg)

            # 接收响应
            resp = json.loads(self.arm_server_client.recv(1024).decode('utf-8'))
            cprint(f"[PoseRecorder] 执行位姿 {name}: {resp}", "green")
            return resp.get("value", False)
        except Exception as e:
            cprint(f"[PoseRecorder] 执行位姿失败: {e}", "red")
            return False

    # ==================== 动作序列 ====================

    def play_sequence(self, sequence_input, is_file=True):
        """
        执行动作序列

        Args:
            sequence_input: 序列JSON文件路径 或 JSON字符串
            is_file: True 表示 sequence_input 是文件路径，False 表示是 JSON 字符串
        """
        try:
            if is_file:
                with open(sequence_input, "r") as f:
                    seq_data = json.load(f)
            else:
                seq_data = json.loads(sequence_input)
        except Exception as e:
            cprint(f"[PoseRecorder] 读取序列失败: {e}", "red")
            return False

        sequence = seq_data.get("sequence", [])
        if not sequence:
            cprint("[PoseRecorder] 序列为空", "red")
            return False

        cprint(f"[PoseRecorder] 开始执行动作序列，共 {len(sequence)} 步", "cyan")

        for i, step in enumerate(sequence):
            cprint(f"\n=== 步骤 {i+1}/{len(sequence)} ===", "yellow")

            # 执行灵巧手（先执行手，再执行臂，或同时）
            hand_gesture = step.get("hand")
            if hand_gesture:
                self.play_hand_gesture(hand_gesture)

            # 执行机械臂
            arm_pose = step.get("arm")
            if arm_pose:
                speed = step.get("speed", 30)
                self.play_pose(arm_pose, speed)

            # 延时
            delay = step.get("delay", 0.5)
            time.sleep(delay)

        cprint("\n[PoseRecorder] 动作序列执行完成", "green")
        return True

    # ==================== 工具方法 ====================

    def list_poses(self):
        """列出所有已录制的位姿"""
        if not self.poses:
            cprint("[PoseRecorder] 没有已录制的位姿", "yellow")
            return

        cprint(f"\n已录制的位姿 ({len(self.poses)} 个):", "cyan")
        print("=" * 60)
        for name, data in self.poses.items():
            desc = data.get("description", "N/A")
            timestamp = data.get("timestamp", "N/A")
            print(f"  {name}: {desc} ({timestamp})")
        print("=" * 60)

    def delete_pose(self, name):
        """删除指定位姿"""
        if name not in self.poses:
            cprint(f"[PoseRecorder] 未找到位姿: {name}", "red")
            return False

        del self.poses[name]
        self._save_poses()
        cprint(f"[PoseRecorder] 已删除位姿: {name}", "green")
        return True

    def interactive_record(self):
        """交互式录制模式"""
        cprint("\n=== 交互式位姿录制模式 ===", "cyan")
        cprint("输入位姿名称开始录制，输入 'q' 退出", "yellow")

        while True:
            print("\n" + "-" * 40)
            name = input("请输入位姿名称: ").strip()

            if name.lower() == 'q':
                break

            if not name:
                cprint("名称不能为空", "red")
                continue

            desc = input("请输入描述（可选）: ").strip()
            self.record_pose(name, desc)

        cprint("\n[PoseRecorder] 退出交互式录制模式", "yellow")


def main():
    parser = argparse.ArgumentParser(
        description="位姿录制与执行脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 录制位姿
  python3 pose_recorder.py record --name home
  python3 pose_recorder.py record --name grasp1 --desc "抓取位置1"

  # 交互式录制
  python3 pose_recorder.py record --interactive

  # 执行位姿
  python3 pose_recorder.py play --name home
  python3 pose_recorder.py play --name grasp1 --speed 50

  # 动作序列（从文件）
  python3 pose_recorder.py sequence --file my_sequence.json

  # 动作序列（从stdin）
  echo '{"sequence": [{"arm": "home", "hand": "open"}]}' | python3 pose_recorder.py sequence

  # 列出/删除位姿
  python3 pose_recorder.py list
  python3 pose_recorder.py delete --name home
        """
    )

    subparsers = parser.add_subparsers(dest="command", help="命令")

    # record 命令
    record_parser = subparsers.add_parser("record", help="录制位姿")
    record_parser.add_argument("--name", "-n", help="位姿名称")
    record_parser.add_argument("--desc", "-d", default="", help="位姿描述")
    record_parser.add_argument("--interactive", "-i", action="store_true", help="交互式录制模式")

    # play 命令
    play_parser = subparsers.add_parser("play", help="执行位姿")
    play_parser.add_argument("--name", "-n", required=True, help="位姿名称")
    play_parser.add_argument("--speed", "-s", type=int, default=30, help="移动速度 (0-100)")

    # sequence 命令
    seq_parser = subparsers.add_parser("sequence", help="执行动作序列")
    seq_parser.add_argument("--file", "-f", help="序列JSON文件路径（不指定则从stdin读取）")

    # list 命令
    subparsers.add_parser("list", help="列出所有已录制的位姿")

    # delete 命令
    del_parser = subparsers.add_parser("delete", help="删除指定位姿")
    del_parser.add_argument("--name", "-n", required=True, help="位姿名称")

    # 全局参数
    parser.add_argument("--pose-file", default="./recorded_poses.json", help="位姿存储文件路径")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    recorder = PoseRecorder(pose_file=args.pose_file)

    try:
        if args.command == "record":
            if args.interactive:
                recorder.interactive_record()
            elif args.name:
                recorder.record_pose(args.name, args.desc)
            else:
                cprint("错误: 请指定 --name 或使用 --interactive 模式", "red")

        elif args.command == "play":
            recorder.play_pose(args.name, args.speed)

        elif args.command == "sequence":
            if args.file:
                recorder.play_sequence(args.file, is_file=True)
            else:
                # 从 stdin 读取
                cprint("[PoseRecorder] 从 stdin 读取序列...", "cyan")
                seq_input = sys.stdin.read().strip()
                if seq_input:
                    recorder.play_sequence(seq_input, is_file=False)
                else:
                    cprint("[PoseRecorder] stdin 为空", "red")

        elif args.command == "list":
            recorder.list_poses()

        elif args.command == "delete":
            recorder.delete_pose(args.name)

    finally:
        # 保持连接，不主动断开
        pass


if __name__ == "__main__":
    main()
