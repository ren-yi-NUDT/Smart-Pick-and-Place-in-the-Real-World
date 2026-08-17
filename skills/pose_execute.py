#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位姿执行技能
============
通过技能框架注册，负责机械臂位姿回放、灵巧手手势执行和动作序列播放。

支持双臂独立控制和并行执行。位姿从 recorded_poses/{left,right}.json 加载，
通过 socket 服务器（左臂 :8010，右臂 :8011）发送控制指令。

命令接口 (run kwargs):
    {"command": "play",  "name": "home",  "arm": "left", "speed": 30}
    {"command": "play",  "name": "home",  "arm": "right", "speed": 30, "block": true}
    {"command": "list",  "arm": "left"}
    {"command": "play",  "sequence": "path/to/sequence.json"}
    {"command": "play",  "sequence": "<json string>", "sequence_is_string": true}
    {"command": "play",  "hand": "open"}
    {"command": "play",  "hand": [0,0,1000,1000,0,0]}
    {"command": "play",  "parallel": [
        {"name": "handover_pose", "arm": "left", "speed": 30},
        {"name": "home", "arm": "right", "speed": 30}
    ]}

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
from threading import Thread
from termcolor import cprint

from skills.base import Skill, register_skill

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

POSES_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "recorded_poses",
)

ARM_SERVER_HOST = "127.0.0.1"
ARM_PORTS = {"left": 8010, "right": 8011}
HAND_SERVER_HOST = "127.0.0.1"
HAND_SERVER_PORT = 8000

# 右臂夹爪（Robotiq 85）TCP 桥接服务
GRIPPER_HOST = "127.0.0.1"
GRIPPER_PORT = 8001
GRIPPER_SRC = "/right_gripper/movement_control"

# 左臂夹爪（与右臂同款 Robotiq 85；左臂灵巧手被替换为夹爪后启用）
LEFT_GRIPPER_HOST = "127.0.0.1"
LEFT_GRIPPER_PORT = 8002
LEFT_GRIPPER_SRC = "/left_gripper/movement_control"

# 6 值手势 → 2 值夹爪指令的映射（左臂换夹爪后保留 open/close 两种语义）
GRIPPER_PRESET_MAP = {
    "open":  [1000, 1000],
    "close": [0, 0],
    # 其他手势（peace/rock/pointing/thumbs_up/ok/grab）无夹爪对应，调用时会告警并落到 close
}

# 轨迹文件目录
TRAJ_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "recorded_trajectories", "right",
)

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

# 强制指定臂的"非录制位姿"动作（轨迹回放直驱 SDK，IP 硬编码）
# 名称 → 唯一允许的臂
ACTION_ARM_RESTRICTIONS = {
    "open_drawer": "right",   # _play_trajectory 硬编码 192.168.1.18
    "close_drawer": "right",
}


# ---------------------------------------------------------------------------
# 辅助函数
# ---------------------------------------------------------------------------

def _load_poses(arm="left"):
    """加载已保存的位姿（从 recorded_poses/{arm}.json）"""
    pose_file = os.path.join(POSES_DIR, f"{arm}.json")
    if os.path.exists(pose_file):
        with open(pose_file, "r") as f:
            return json.load(f)
    return {}


def _load_trajectory(name):
    """加载已录制的轨迹（从 recorded_trajectories/right/{name}.json）"""
    traj_file = os.path.join(TRAJ_DIR, f"{name}.json")
    if not os.path.exists(traj_file):
        cprint(f"[pose_execute] 轨迹不存在: {traj_file}", "red")
        return None
    with open(traj_file, "r") as f:
        return json.load(f)


def _resolve_action_arm(name, requested_arm):
    """解析动作 `name` 是否能在 `requested_arm` 上执行。

    路由规则：
      1. 在 ACTION_ARM_RESTRICTIONS 中的动作（如 open_drawer）→ 强制指定臂
      2. 否则查 recorded_poses/{arm}.json：
         - 命中 requested_arm → 通过
         - 命中另一臂 → 报错并提示正确臂
         - 都未命中 → 报错"未找到位姿"

    Returns:
        (ok, msg, authoritative_arm)
        - ok=True,  msg=None,    authoritative_arm=<可执行的臂>
        - ok=False, msg=<原因>,  authoritative_arm=<正确臂 or None>
    """
    requested_arm = (requested_arm or "left").lower()
    if requested_arm not in ("left", "right"):
        return False, (
            f"无效的臂参数 '{requested_arm}'（应为 left 或 right）"
        ), None

    if name in ACTION_ARM_RESTRICTIONS:
        required = ACTION_ARM_RESTRICTIONS[name]
        if requested_arm == required:
            return True, None, required
        return False, (
            f"动作 '{name}' 只能由 {required}臂执行"
            f"（请求的是 {requested_arm}臂）"
        ), required

    if _load_poses(requested_arm).get(name):
        return True, None, requested_arm

    other_arm = "right" if requested_arm == "left" else "left"
    if _load_poses(other_arm).get(name):
        return False, (
            f"位姿 '{name}' 只能由 {other_arm}臂执行"
            f"（请求的是 {requested_arm}臂）"
        ), other_arm

    return False, (
        f"未找到位姿 '{name}'（左/右臂录制库均无此条目）"
    ), None


def _send_gripper_cmd(values):
    """通过 TCP 发送夹爪控制指令（静默失败）。"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        sock.connect((GRIPPER_HOST, GRIPPER_PORT))
        sock.sendall(json.dumps({
            "src": GRIPPER_SRC, "type": "set", "cmd": list(values),
        }).encode())
        sock.recv(256)
        sock.close()
        return True
    except Exception:
        return False


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


def _send_arm_command(sock, joint_angles, speed=50, block=True):
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
    位姿执行技能（双臂 + 并行）

    通过 socket 服务器回放已录制的机械臂位姿、执行灵巧手手势、播放动作序列。
    支持通过 arm 参数指定左/右臂，以及通过 parallel 参数并行执行多个动作。

    使用:
        skill = PoseExecuteSkill()
        skill.run(command="play", name="home", arm="left", speed=30)
        skill.run(command="play", name="home", arm="right", speed=30)
        skill.run(command="list", arm="left")
        skill.run(command="play", hand="open")
        skill.run(command="play", sequence="sequence.json")
        skill.run(command="play", parallel=[
            {"name": "handover_pose", "arm": "left"},
            {"name": "home", "arm": "right"},
        ])
    """

    def __init__(self, **kw):
        super().__init__(**kw)
        self._arm_socks = {}   # {"left": sock, "right": sock}
        self._hand_sock = None

    # -- 连接管理 --

    def _get_arm_sock(self, arm="left"):
        """获取指定臂的 socket 连接"""
        if arm not in self._arm_socks or self._arm_socks[arm] is None:
            port = ARM_PORTS.get(arm, 8010)
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(120)
                sock.connect((ARM_SERVER_HOST, port))
                self._arm_socks[arm] = sock
                cprint(
                    f"[pose_execute] 已连接 {arm} 臂服务 {ARM_SERVER_HOST}:{port}",
                    "green",
                )
            except Exception as e:
                cprint(f"[pose_execute] 连接 {arm} 臂服务失败: {e}", "red")
                self._arm_socks[arm] = None
        return self._arm_socks[arm]

    @property
    def hand_sock(self):
        """获取左臂末端执行器 socket 连接（按 hand_type 分发到夹爪或灵巧手）。

        连接目标由 robot_config.json 的 arms.left.hand_type 决定：
        - "gripper"  → LEFT_GRIPPER_PORT (8002)
        - "dexterous" → HAND_SERVER_PORT (8000, 旧灵巧手)
        """
        if self._hand_sock is None:
            try:
                left_cfg = self.config.get_arm_config("left") if self.config._is_new_format else {}
                hand_type = left_cfg.get("hand_type", "dexterous")
                if hand_type == "gripper":
                    host, port = LEFT_GRIPPER_HOST, LEFT_GRIPPER_PORT
                    label = "左臂夹爪"
                else:
                    host, port = HAND_SERVER_HOST, HAND_SERVER_PORT
                    label = "灵巧手"
                self._hand_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._hand_sock.connect((host, port))
                cprint(
                    f"[pose_execute] 已连接{label}服务 {host}:{port}",
                    "green",
                )
            except Exception as e:
                cprint(f"[pose_execute] 连接左臂末端执行器服务失败: {e}", "red")
                self._hand_sock = None
        return self._hand_sock

    def _left_hand_type(self):
        """Return left arm's hand_type ("gripper" or "dexterous")."""
        if self.config._is_new_format:
            return self.config.get_arm_config("left").get("hand_type", "dexterous")
        return "dexterous"

    # -- 核心: run() --

    def run(self, **kwargs):
        """
        执行位姿命令

        Args:
            **kwargs:
                command (str): "play" 或 "list"
                name (str): 位姿名称
                arm (str): "left" 或 "right" (默认 "left")
                speed (int): 移动速度 0-100 (默认 50)
                block (bool): 是否阻塞等待 (默认 True)
                sequence (str): 序列文件路径或 JSON 字符串
                sequence_is_string (bool): sequence 是 JSON 字符串而非文件路径
                hand: 手势输入 (预设名/数组/字典)
                parallel (list): 并行动作列表，每项含 name/arm/speed 等

        Returns:
            dict: {"success": bool, "info": str}
        """
        command = kwargs.get("command", "play")

        if command == "list":
            arm = kwargs.get("arm", "left")
            return self.list_poses(arm)

        elif command == "play":
            results = {}

            # 并行执行
            parallel = kwargs.get("parallel")
            if parallel is not None:
                results["parallel"] = self.play_parallel(parallel)

            # 播放灵巧手手势
            hand = kwargs.get("hand")
            if hand is not None:
                results["hand"] = self.play_hand_gesture(hand)

            # 播放动作序列
            sequence = kwargs.get("sequence")
            if sequence is not None:
                is_string = kwargs.get("sequence_is_string", False)
                results["sequence"] = self.play_sequence(sequence, is_file=not is_string)

            # 播放单个位姿（或轨迹回放）
            name = kwargs.get("name")
            if name is not None:
                arm = kwargs.get("arm", "left")
                # 抽屉轨迹：硬编码右臂，先做路由检查
                if name in ACTION_ARM_RESTRICTIONS and arm != ACTION_ARM_RESTRICTIONS[name]:
                    required = ACTION_ARM_RESTRICTIONS[name]
                    msg = (
                        f"动作 '{name}' 只能由 {required}臂执行"
                        f"（请求的是 {arm}臂）"
                    )
                    cprint(f"[pose_execute] 路由失败: {msg}", "red")
                    results["pose"] = False
                    results["pose_error"] = msg
                elif name == "open_drawer":
                    results["pose"] = self.play_open_drawer(speed=1.5)
                elif name == "close_drawer":
                    results["pose"] = self.play_close_drawer(speed=1.5)
                else:
                    speed = kwargs.get("speed", 50)
                    block = kwargs.get("block", True)
                    results["pose"] = self.play_pose(name, arm, speed, block)

            if not results:
                return {"success": False, "info": "未指定 name/sequence/hand/parallel 参数"}

            return {
                "success": all(r for r in results.values() if isinstance(r, bool)),
                "info": results,
            }

        elif command == "open_drawer":
            arm = kwargs.get("arm", "right")
            if arm != "right":
                msg = f"动作 'open_drawer' 只能由右臂执行（请求的是 {arm}臂）"
                cprint(f"[pose_execute] 路由失败: {msg}", "red")
                return {"success": False, "info": msg}
            speed = kwargs.get("speed", 1.5)
            ok = self.play_open_drawer(speed=speed)
            return {"success": ok, "info": "开抽屉" if ok else "开抽屉失败"}

        elif command == "close_drawer":
            arm = kwargs.get("arm", "right")
            if arm != "right":
                msg = f"动作 'close_drawer' 只能由右臂执行（请求的是 {arm}臂）"
                cprint(f"[pose_execute] 路由失败: {msg}", "red")
                return {"success": False, "info": msg}
            speed = kwargs.get("speed", 1.5)
            ok = self.play_close_drawer(speed=speed)
            return {"success": ok, "info": "关抽屉" if ok else "关抽屉失败"}

        else:
            return {"success": False, "info": f"未知命令: {command}"}

    # -- 位姿播放 --

    def play_pose(self, name, arm="left", speed=50, block=True):
        """
        执行预设位姿（自动路由到正确的臂）

        Args:
            name: 位姿名称或受控动作名（open_drawer/close_drawer）
            arm: "left" 或 "right"（仅当位姿存在于此臂时通过；
                 若位姿只存在于另一臂或受 ACTION_ARM_RESTRICTIONS 限制，
                 会拒绝执行并打印路由错误）
            speed: 移动速度 (0-100)
            block: 是否阻塞等待完成

        Returns:
            bool: 是否执行成功
        """
        ok, msg, authoritative_arm = _resolve_action_arm(name, arm)
        if not ok:
            cprint(f"[pose_execute] 路由失败: {msg}", "red")
            return False
        arm = authoritative_arm

        # 受控动作（轨迹回放，硬编码到右臂 SDK）
        if name in ACTION_ARM_RESTRICTIONS:
            if name == "open_drawer":
                return self.play_open_drawer(speed=speed / 20 if speed else 1.5)
            if name == "close_drawer":
                return self.play_close_drawer(speed=speed / 20 if speed else 1.5)

        poses = _load_poses(arm)
        pose_data = poses[name]
        joint_angles = pose_data["joint_angles_deg"]

        sock = self._get_arm_sock(arm)
        if sock is None:
            cprint(f"[pose_execute] [{arm}] 无可用连接", "red")
            return False

        try:
            resp = _send_arm_command(sock, joint_angles, speed, block)
            cprint(f"[pose_execute] [{arm}] 执行位姿 {name}: {resp}", "green")
            return resp.get("value", False)
        except Exception as e:
            cprint(f"[pose_execute] [{arm}] 执行位姿失败: {e}", "red")
            self._arm_socks[arm] = None
            return False

    # -- 并行执行 --

    def play_parallel(self, actions):
        """
        并行执行多个动作

        Args:
            actions: 动作列表，每项是一个 dict，支持:
                - {"name": "pose_name", "arm": "left", "speed": 30}
                - {"hand": "open"}
                - {"name": "pose_name", "arm": "right", "speed": 20}

        Returns:
            bool: 是否全部执行成功
        """
        if not actions:
            cprint("[pose_execute] parallel 动作列表为空", "red")
            return False

        cprint(f"[pose_execute] 并行执行 {len(actions)} 个动作", "cyan")
        results = [None] * len(actions)

        def _execute(idx, action):
            name = action.get("name")
            hand = action.get("hand")
            if name is not None:
                arm = action.get("arm", "left")
                speed = action.get("speed", 50)
                results[idx] = self.play_pose(name, arm, speed, block=True)
            elif hand is not None:
                results[idx] = self.play_hand_gesture(hand)

        threads = []
        for i, action in enumerate(actions):
            t = Thread(target=_execute, args=(i, action))
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        all_ok = all(r for r in results if isinstance(r, bool))
        cprint(
            f"[pose_execute] 并行执行完成: {results}",
            "green" if all_ok else "red",
        )
        return all_ok

    # -- 灵巧手手势 --

    def play_hand_gesture(self, hand_input):
        """
        执行手势

        Args:
            hand_input: 手势输入（预设名/数组/字典）

        Returns:
            bool: 是否执行成功

        Notes:
            左臂末端执行器类型由 robot_config.json 的 arms.left.hand_type 决定。
            - 灵巧手模式：发送 6 值手势数组（如 peace/thumbs_up 等预设）
            - 夹爪模式：仅支持 "open" / "close" 两种语义，其他预设会被映射到 close
              并打印警告（夹爪物理上无法表达多指手势）
        """
        if self._left_hand_type() == "gripper":
            return self._play_left_gripper(hand_input)
        return self._play_dexterous_gesture(hand_input)

    def _play_left_gripper(self, hand_input):
        """夹爪模式：把预设名/数组映射到 2 值指令。"""
        if isinstance(hand_input, str):
            key = hand_input.lower()
            if key in GRIPPER_PRESET_MAP:
                cmd_values = list(GRIPPER_PRESET_MAP[key])
            else:
                cprint(
                    f"[pose_execute] 左臂为夹爪，手势 '{hand_input}' 不支持，"
                    f"回落到 close",
                    "yellow",
                )
                cmd_values = [0, 0]
        elif isinstance(hand_input, (list, tuple)):
            if len(hand_input) >= 2:
                # 假设前两位是 [thumb, index] 类的粗略映射，取均值
                avg = int(sum(hand_input[:2]) / 2)
                cmd_values = [avg, avg]
            else:
                cmd_values = [0, 0]
        else:
            cprint(f"[pose_execute] 不支持的手势格式: {type(hand_input)}", "red")
            return False

        sock = self.hand_sock
        if sock is None:
            return False
        try:
            cmd = {"src": LEFT_GRIPPER_SRC, "type": "set", "cmd": cmd_values}
            sock.sendall(json.dumps(cmd).encode("utf-8"))
            resp = json.loads(sock.recv(1024).decode("utf-8"))
            label = hand_input if isinstance(hand_input, str) else cmd_values
            cprint(f"[pose_execute] 执行左臂夹爪 {label}: {resp}", "green")
            return resp.get("value", False) in (True, [1000, 1000], [0, 0])
        except Exception as e:
            cprint(f"[pose_execute] 左臂夹爪指令失败: {e}", "red")
            self._hand_sock = None
            return False

    def _play_dexterous_gesture(self, hand_input):
        """灵长手模式：原 6 值手势路径。"""
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

        序列中每步支持:
            - "arm_pose": 位姿名称（旧格式，默认左臂）
            - "name": 位姿名称 + "arm": "left"/"right"
            - "hand": 手势
            - "parallel": 并行动作列表
            - "speed": 速度
            - "delay": 延时
        """
        try:
            if isinstance(sequence_input, (list, dict)):
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

            # 并行子步骤
            parallel = step.get("parallel")
            if parallel is not None:
                if not self.play_parallel(parallel):
                    all_ok = False

            # 灵巧手 (先执行手)
            hand_gesture = step.get("hand")
            if hand_gesture is not None:
                if not self.play_hand_gesture(hand_gesture):
                    all_ok = False

            # 机械臂（支持新旧格式）
            arm_pose = step.get("arm_pose") or step.get("name")
            if arm_pose is not None:
                arm = step.get("arm", "left")
                speed = step.get("speed", 50)
                if not self.play_pose(arm_pose, arm, speed):
                    all_ok = False

            delay = step.get("delay", 0.2)
            time.sleep(delay)

        cprint("\n[pose_execute] 动作序列执行完成", "green")
        return all_ok

    # -- 查询 --

    def list_poses(self, arm="left"):
        """列出已录制的位姿"""
        poses = _load_poses(arm)

        if not poses:
            cprint(f"[pose_execute] [{arm}] 没有已录制的位姿", "yellow")
            return {"success": True, "info": f"{arm}臂没有已录制的位姿", "poses": {}}

        cprint(f"\n[{arm}臂] 已录制的位姿 ({len(poses)} 个):", "cyan")
        print("=" * 60)
        for name, data in poses.items():
            desc = data.get("description", "N/A")
            timestamp = data.get("timestamp", "N/A")
            print(f"  {name}: {desc} ({timestamp})")
        print("=" * 60)

        return {
            "success": True,
            "info": f"{arm}臂共 {len(poses)} 个位姿",
            "poses": list(poses.keys()),
        }

    # -- 轨迹回放（开关抽屉，使用 SDK 直驱 CAN FD，保证流畅度） --

    def _play_trajectory(self, name, speed=1.5):
        """回放录制的轨迹（臂 + 夹爪），使用 Robotic_Arm SDK 直接驱动。

        Args:
            name: 轨迹名（不含路径和扩展名）
            speed: 回放速度倍率

        Returns:
            bool: 是否回放成功
        """
        if self.config.sim_mode:
            return self._play_trajectory_sim(name, speed=speed)

        from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

        ARM_IP = "192.168.1.18"
        ARM_SDK_PORT = 8080

        t = _load_trajectory(name)
        if t is None:
            return False

        waypoints = t["waypoints"]
        has_gripper = t.get("recorded_gripper", False)

        if len(waypoints) < 2:
            cprint("[pose_execute] 轨迹点不足", "red")
            return False

        total_dt_ms = waypoints[-1][0] - waypoints[0][0]
        num_intervals = len(waypoints) - 1
        base_interval_s = (total_dt_ms / num_intervals) / 1000.0
        adjusted_interval_s = max(0.005, base_interval_s / speed)

        cprint(
            f"[pose_execute] 回放轨迹 {name}: "
            f"{t['duration_ms'] / 1000:.1f}s × {speed}x, "
            f"{len(waypoints)} 航点, 间隔 {adjusted_interval_s * 1000:.0f}ms (SDK)",
            "cyan",
        )

        # 连接右臂 SDK
        arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
        handle = arm.rm_create_robot_arm(ARM_IP, ARM_SDK_PORT, level=3)
        if handle.id == -1:
            cprint("[pose_execute] SDK 连接右臂失败", "red")
            return False
        cprint(f"[pose_execute] SDK 已连接右臂, 句柄 ID: {handle.id}", "green")

        # 运动到起点
        start_joints = t["start_joint_deg"]
        tag = arm.rm_movej(joint=start_joints, v=20, r=0, connect=0, block=1)
        if tag != 0:
            cprint(f"[pose_execute] 运动到起点失败 tag={tag}", "red")
            return False

        # 起始夹爪
        if has_gripper and len(waypoints[0]) >= 10:
            _send_gripper_cmd(waypoints[0][8:10])

        prev_gripper = waypoints[0][8:10] if has_gripper and len(waypoints[0]) >= 10 else None

        try:
            for i, wp in enumerate(waypoints):
                joint = wp[1:8]
                arm.rm_movej_canfd(joint=joint, follow=False, expand=0)

                # 夹爪状态变化时发送指令
                if has_gripper and len(wp) >= 10:
                    gv = wp[8:10]
                    if gv != prev_gripper:
                        _send_gripper_cmd(gv)
                        prev_gripper = gv

                if (i + 1) % max(1, len(waypoints) // 10) == 0:
                    cprint(
                        f"[pose_execute] 轨迹 {name}: {i + 1}/{len(waypoints)} "
                        f"({(i + 1) * 100 // len(waypoints)}%)",
                        "yellow",
                    )

                time.sleep(adjusted_interval_s)

            cprint(f"[pose_execute] 轨迹 {name} 回放完成 ✓", "green")
            return True

        except KeyboardInterrupt:
            cprint(f"\n[pose_execute] 轨迹 {name} 用户中断", "yellow")
            return False

    def _play_trajectory_sim(self, name, speed=1.5):
        """回放右臂录制轨迹到 PyBullet SimServer（sim 模式）。

        抽屉轨迹固定由右臂执行。关节航点批量通过 ``execute_trajectory`` 下发，
        夹爪状态在航点间变化时分段回放，以还原"抓把手→拉开→松手"的时序。
        """
        t = _load_trajectory(name)
        if t is None:
            return False

        waypoints = t["waypoints"]
        has_gripper = t.get("recorded_gripper", False)
        if len(waypoints) < 2:
            cprint("[pose_execute] 轨迹点不足", "red")
            return False

        from core.sim_arm import SimArmClient
        from core.sim_gripper import SimGripperClient
        host = self.config.shared.get("host", "127.0.0.1")
        arm = SimArmClient(host, 8031, side="right")
        if not arm.connect():
            return False
        gripper = None
        if has_gripper:
            gripper = SimGripperClient(host, 8031, src="/right_gripper/movement_control")
            gripper.connect()

        cprint(
            f"[pose_execute] (sim) 回放右臂轨迹 {name}: "
            f"{len(waypoints)} 航点 × {speed}x",
            "cyan",
        )

        def _set_gripper(v):
            if gripper is None:
                return
            action = "open" if v[0] > 500 else "close"
            gripper._send({"cmd": "gripper", "side": "right",
                           "action": action, "value": int(v[0])})

        # 起始夹爪
        if has_gripper:
            _set_gripper(waypoints[0][8:10])

        # 按夹爪状态分段回放关节轨迹
        seg = []
        prev_gv = tuple(waypoints[0][8:10]) if has_gripper else None
        for wp in waypoints:
            gv = tuple(wp[8:10]) if has_gripper else None
            if has_gripper and gv != prev_gv:
                if seg:
                    arm.execute_trajectory(seg, speed=20)
                    seg = []
                _set_gripper(gv)
                prev_gv = gv
            seg.append(list(wp[1:8]))
        if seg:
            arm.execute_trajectory(seg, speed=20)

        cprint(f"[pose_execute] 轨迹 {name} 回放完成 ✓", "green")
        return True

    def play_open_drawer(self, speed=1.5):
        """开抽屉：home → 抓把手 → 拉开 → 松手 → 回 home。

        Args:
            speed: 回放速度倍率 (默认 1.5x)

        Returns:
            bool: 是否成功
        """
        return self._play_trajectory("open_drawer", speed=speed)

    def play_close_drawer(self, speed=1.5):
        """关抽屉：home → 推关 → 回 home（不抓把手）。

        Args:
            speed: 回放速度倍率 (默认 1.5x)

        Returns:
            bool: 是否成功
        """
        return self._play_trajectory("close_drawer", speed=speed)
