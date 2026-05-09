#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位姿执行技能
============
通过技能框架注册，负责机械臂位姿回放、灵巧手手势执行和动作序列播放。

使用 socket 服务器 (arm :8010, hand :8000) 发送控制指令。
从 recorded_poses.json 读取已录制的位姿数据。

命令接口 (run kwargs):
    {"command": "play",  "name": "home",  "speed": 30}
    {"command": "play",  "name": "home",  "speed": 30, "block": true}
    {"command": "list"}
    {"command": "play",  "sequence": "path/to/sequence.json"}
    {"command": "play",  "sequence": "<json string>", "sequence_is_string": true}
    {"command": "play",  "hand": "open"}
    {"command": "play",  "hand": [0,0,1000,1000,0,0]}

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
import time
from termcolor import cprint

from skills.base import Skill, register_skill

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

DEFAULT_POSE_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "recorded_poses.json",
)

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


# ---------------------------------------------------------------------------
# 辅助函数
# ---------------------------------------------------------------------------

def _load_poses(pose_file=None):
    """加载已保存的位姿"""
    pose_file = pose_file or DEFAULT_POSE_FILE
    if os.path.exists(pose_file):
        with open(pose_file, "r") as f:
            return json.load(f)
    return {}


def _parse_hand_gesture(hand_input):
    """
    解析手势输入，返回 6 值数组

    支持格式:
    - 预设名称: "open", "close", "peace", "rock", "pointing", "thumbs_up", "ok", "grab"
    - 数组: [0, 0, 1000, 1000, 0, 0]
    - 字典: {"pinky": 0, "ring": 0, "middle": 1000, "index": 1000, "thumb": 0, "thumb_abduct": 0}
    """
    if isinstance(hand_input, str):
        if hand_input in HAND_GESTURES:
            return HAND_GESTURES[hand_input]
        else:
            cprint(f"[pose_execute] 未知手势预设: {hand_input}", "red")
            return None
    elif isinstance(hand_input, list):
        if len(hand_input) == 6:
            return hand_input
        else:
            cprint(f"[pose_execute] 手指数组长度必须为6，收到: {len(hand_input)}", "red")
            return None
    elif isinstance(hand_input, dict):
        finger_map = ["pinky", "ring", "middle", "index", "thumb", "thumb_abduct"]
        result = []
        for finger in finger_map:
            result.append(hand_input.get(finger, 0))
        return result
    else:
        cprint(f"[pose_execute] 不支持的手势格式: {type(hand_input)}", "red")
        return None


def _send_arm_command(sock, joint_angles, speed=30, block=True):
    """
    通过 socket 发送机械臂关节运动指令

    Args:
        sock: 已连接的 socket
        joint_angles: 关节角度列表 (度)
        speed: 移动速度 0-100
        block: 是否阻塞等待完成

    Returns:
        dict: 服务器响应
    """
    joint_dict = {f"J{i+1}": j for i, j in enumerate(joint_angles)}

    cmd = {
        "srv": "/right_arm/movement_control",
        "cmd": [
            {"type": "start", "act": []},
            {"type": "js", "act": joint_dict, "speed": speed, "block": block},
            {"type": "end", "act": []},
        ],
    }
    msg = json.dumps(cmd).encode("utf-8")

    # 4 字节大端长度前缀 + JSON 数据
    length_prefix = struct.pack(">I", len(msg))
    sock.sendall(length_prefix + msg)

    resp = json.loads(sock.recv(1024).decode("utf-8"))
    return resp


def _send_hand_command(sock, gesture):
    """
    通过 socket 发送灵巧手手势指令

    Args:
        sock: 已连接的 socket
        gesture: 6 值数组

    Returns:
        dict: 服务器响应
    """
    cmd = {
        "src": "/left_hand/movement_control",
        "type": "set",
        "cmd": gesture,
    }
    msg = json.dumps(cmd).encode("utf-8")
    sock.sendall(msg)

    resp = json.loads(sock.recv(1024).decode("utf-8"))
    return resp


# ---------------------------------------------------------------------------
# 技能类
# ---------------------------------------------------------------------------

@register_skill("pose_execute")
class PoseExecuteSkill(Skill):
    """
    位姿执行技能

    通过 socket 服务器回放已录制的机械臂位姿、执行灵巧手手势、播放动作序列。

    使用:
        skill = PoseExecuteSkill()
        skill.run(command="play", name="home", speed=30)
        skill.run(command="list")
        skill.run(command="play", hand="open")
        skill.run(command="play", sequence="sequence.json")
    """

    def __init__(self, **kw):
        super().__init__(**kw)
        # self.arm and self.hand are injected by the skill framework (socket connections)
        # If not injected, connections are created on first use
        self._arm_sock = None
        self._hand_sock = None

    # -- 连接管理 (fallback) --

    @property
    def arm_sock(self):
        """获取机械臂 socket 连接 (独立的 raw socket，不走 base.py 的 ArmClient)"""
        if self._arm_sock is None:
            try:
                self._arm_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._arm_sock.connect((ARM_SERVER_HOST, ARM_SERVER_PORT))
                cprint(
                    f"[pose_execute] 已连接机械臂服务 {ARM_SERVER_HOST}:{ARM_SERVER_PORT}",
                    "green",
                )
            except Exception as e:
                cprint(f"[pose_execute] 连接机械臂服务失败: {e}", "red")
                self._arm_sock = None
        return self._arm_sock

    @property
    def hand_sock(self):
        """获取灵巧手 socket 连接 (独立的 raw socket，不走 base.py 的 HandClient)"""
        if self._hand_sock is None:
            try:
                self._hand_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._hand_sock.connect((HAND_SERVER_HOST, HAND_SERVER_PORT))
                cprint(
                    f"[pose_execute] 已连接灵巧手服务 {HAND_SERVER_HOST}:{HAND_SERVER_PORT}",
                    "green",
                )
            except Exception as e:
                cprint(f"[pose_execute] 连接灵巧手服务失败: {e}", "red")
                self._hand_sock = None
        return self._hand_sock

    # -- 核心: run() --

    def run(self, **kwargs):
        """
        执行位姿命令

        Args:
            **kwargs:
                command (str): "play" 或 "list"
                name (str): 位姿名称 (command="play" 时用于播放单个位姿)
                speed (int): 移动速度 0-100 (默认 30)
                block (bool): 是否阻塞等待 (默认 True)
                sequence (str): 序列文件路径或 JSON 字符串
                sequence_is_string (bool): sequence 是 JSON 字符串而非文件路径
                hand: 手势输入 (预设名/数组/字典)

        Returns:
            dict: {"success": bool, "info": str}
        """
        command = kwargs.get("command", "play")

        if command == "list":
            return self.list_poses()

        elif command == "play":
            results = {}

            # 播放灵巧手手势
            hand = kwargs.get("hand")
            if hand is not None:
                results["hand"] = self.play_hand_gesture(hand)

            # 播放动作序列
            sequence = kwargs.get("sequence")
            if sequence is not None:
                is_string = kwargs.get("sequence_is_string", False)
                results["sequence"] = self.play_sequence(sequence, is_file=not is_string)

            # 播放单个位姿
            name = kwargs.get("name")
            if name is not None:
                speed = kwargs.get("speed", 30)
                block = kwargs.get("block", True)
                results["pose"] = self.play_pose(name, speed, block)

            if not results:
                return {"success": False, "info": "未指定 name/sequence/hand 参数"}

            return {
                "success": all(r for r in results.values() if isinstance(r, bool)),
                "info": results,
            }

        else:
            return {"success": False, "info": f"未知命令: {command}"}

    # -- 位姿播放 --

    def play_pose(self, name, speed=30, block=True):
        """
        执行预设位姿

        Args:
            name: 位姿名称
            speed: 移动速度 (0-100)
            block: 是否阻塞等待完成

        Returns:
            bool: 是否执行成功
        """
        poses = _load_poses()

        if name not in poses:
            cprint(f"[pose_execute] 未找到位姿: {name}", "red")
            return False

        pose_data = poses[name]
        joint_angles = pose_data["joint_angles_deg"]

        sock = self.arm_sock
        if sock is None:
            cprint("[pose_execute] 无可用机械臂连接", "red")
            return False

        try:
            resp = _send_arm_command(sock, joint_angles, speed, block)
            cprint(f"[pose_execute] 执行位姿 {name}: {resp}", "green")
            return resp.get("value", False)
        except Exception as e:
            cprint(f"[pose_execute] 执行位姿失败: {e}", "red")
            # 重置连接以便下次重连
            self._arm_sock = None
            return False

    # -- 灵巧手手势 --

    def play_hand_gesture(self, hand_input):
        """
        执行手势

        Args:
            hand_input: 手势输入（预设名/数组/字典）

        Returns:
            bool: 是否执行成功
        """
        gesture = _parse_hand_gesture(hand_input)
        if gesture is None:
            return False

        sock = self.hand_sock
        if sock is None:
            cprint("[pose_execute] 无可用灵巧手连接", "red")
            return False

        try:
            resp = _send_hand_command(sock, gesture)
            label = hand_input if isinstance(hand_input, str) else gesture
            cprint(f"[pose_execute] 执行手势 {label}: {resp}", "green")
            return resp.get("value", False)
        except Exception as e:
            cprint(f"[pose_execute] 执行手势失败: {e}", "red")
            self._hand_sock = None
            return False

    # -- 动作序列 --

    def play_sequence(self, sequence_input, is_file=True):
        """
        执行动作序列

        Args:
            sequence_input: 序列 JSON 文件路径、JSON 字符串，或直接的列表/字典
            is_file: True 表示 sequence_input 是文件路径，False 表示是 JSON 字符串或直接数据

        Returns:
            bool: 是否全部执行成功
        """
        try:
            if isinstance(sequence_input, (list, dict)):
                # 已经是解析好的数据
                seq_data = sequence_input if isinstance(sequence_input, dict) else {"sequence": sequence_input}
            elif is_file:
                with open(sequence_input, "r") as f:
                    seq_data = json.load(f)
            elif isinstance(sequence_input, str):
                seq_data = json.loads(sequence_input)
                if isinstance(seq_data, list):
                    seq_data = {"sequence": seq_data}
        except Exception as e:
            cprint(f"[pose_execute] 读取序列失败: {e}", "red")
            return False

        sequence = seq_data.get("sequence", [])
        if not sequence:
            cprint("[pose_execute] 序列为空", "red")
            return False

        cprint(f"[pose_execute] 开始执行动作序列，共 {len(sequence)} 步", "cyan")

        all_ok = True
        for i, step in enumerate(sequence):
            cprint(f"\n=== 步骤 {i+1}/{len(sequence)} ===", "yellow")

            # 执行灵巧手 (先执行手)
            hand_gesture = step.get("hand")
            if hand_gesture is not None:
                if not self.play_hand_gesture(hand_gesture):
                    all_ok = False

            # 执行机械臂
            arm_pose = step.get("arm")
            if arm_pose is not None:
                speed = step.get("speed", 30)
                if not self.play_pose(arm_pose, speed):
                    all_ok = False

            # 延时
            delay = step.get("delay", 0.5)
            time.sleep(delay)

        cprint("\n[pose_execute] 动作序列执行完成", "green")
        return all_ok

    # -- 查询 --

    def list_poses(self):
        """列出所有已录制的位姿"""
        poses = _load_poses()

        if not poses:
            cprint("[pose_execute] 没有已录制的位姿", "yellow")
            return {"success": True, "info": "没有已录制的位姿", "poses": {}}

        cprint(f"\n已录制的位姿 ({len(poses)} 个):", "cyan")
        print("=" * 60)
        for name, data in poses.items():
            desc = data.get("description", "N/A")
            timestamp = data.get("timestamp", "N/A")
            print(f"  {name}: {desc} ({timestamp})")
        print("=" * 60)

        return {
            "success": True,
            "info": f"共 {len(poses)} 个位姿",
            "poses": list(poses.keys()),
        }
