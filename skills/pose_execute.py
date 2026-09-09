#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位姿执行技能
============
通过技能框架注册，负责机械臂位姿回放、夹爪控制和动作序列播放。

支持双臂独立控制和并行执行。位姿从 recorded_poses/{left,right}.json 加载，
通过 socket 服务器（左臂 :8010，右臂 :8011）发送控制指令。

命令接口 (run kwargs):
    {"command": "play",  "name": "home",  "arm": "left", "speed": 30}
    {"command": "play",  "name": "home",  "arm": "right", "speed": 30, "block": true}
    {"command": "list",  "arm": "left"}
    {"command": "play",  "sequence": "path/to/sequence.json"}
    {"command": "play",  "sequence": "<json string>", "sequence_is_string": true}
    {"command": "play",  "hand": "open", "arm": "left"}
    {"command": "play",  "hand": "open", "arm": "right"}
    {"command": "play",  "parallel": [
        {"name": "handover_pose", "arm": "left", "speed": 30},
        {"name": "home", "arm": "right", "speed": 30}
    ]}

夹爪（Robotiq 85）仅支持 "open" / "close" 两种语义。
"""

import os
import json
import socket
import time
from threading import Thread
from termcolor import cprint

from skills.base import Skill, register_skill
from core.drawer_executor import DRAWER_TRAJECTORY_SPEED
from core.tcp_protocol import JSON_FRAME_PROTOCOL, recv_json_compat, send_json_frame

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

POSES_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "recorded_poses",
)

ARM_SERVER_HOST = "127.0.0.1"
ARM_PORTS = {"left": 8010, "right": 8011}

# 6 值手势 → 2 值夹爪指令的映射（左臂换夹爪后保留 open/close 两种语义）
GRIPPER_PRESET_MAP = {
    "open":  [1000, 1000],
    "close": [0, 0],
    # 其他手势（peace/rock/pointing/thumbs_up/ok/grab）无夹爪对应，调用时会告警并落到 close
}

# 强制指定臂的非录制动作（抽屉执行委托给统一 real/sim 执行器）
# 名称 → 唯一允许的臂
ACTION_ARM_RESTRICTIONS = {
    "open_drawer": "right",   # drawer executor routes through the right bridge
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


def _resolve_drawer_speed(speed=None):
    """Normalize drawer speed across every pose_execute entry point.

    Drawer actions use trajectory multipliers (0.5 means half speed), while
    generic pose commands historically use controller speeds (0..100, with
    50 as the default).  OpenClaw can pass that generic default when it
    selects ``command=play``; normalize controller-style values so they are
    never accidentally interpreted as a 50x trajectory multiplier.
    """
    if speed is None:
        return DRAWER_TRAJECTORY_SPEED
    value = float(speed)
    if value == 50:
        return DRAWER_TRAJECTORY_SPEED
    if value > 5:
        return value / 20
    return value


def _send_arm_command(sock, joint_angles, arm="right", speed=50, block=True):
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

    if arm not in ("left", "right"):
        raise ValueError(f"不支持的机械臂: {arm}")

    cmd = {
        "srv": f"/{arm}_arm/movement_control",
        "cmd": [
            {"type": "start", "act": []},
            {"type": "js", "act": joint_dict, "speed": speed, "block": block},
            {"type": "end", "act": []},
        ],
    }
    cmd["_tcp_protocol"] = JSON_FRAME_PROTOCOL
    send_json_frame(sock, cmd)
    resp = recv_json_compat(sock)
    return resp


# ---------------------------------------------------------------------------
# 技能类
# ---------------------------------------------------------------------------

@register_skill("pose_execute")
class PoseExecuteSkill(Skill):
    """
    位姿执行技能（双臂 + 并行）

    通过 socket 服务器回放已录制的机械臂位姿、执行夹爪动作、播放动作序列。
    支持通过 arm 参数指定左/右臂，以及通过 parallel 参数并行执行多个动作。

    使用:
        skill = PoseExecuteSkill()
        skill.run(command="play", name="home", arm="left", speed=30)
        skill.run(command="play", name="home", arm="right", speed=30)
        skill.run(command="list", arm="left")
        skill.run(command="play", hand="open", arm="left")
        skill.run(command="play", hand="open", arm="right")
        skill.run(command="play", sequence="sequence.json")
        skill.run(command="play", parallel=[
            {"name": "handover_pose", "arm": "left"},
            {"name": "home", "arm": "right"},
        ])
    """

    def __init__(self, **kw):
        super().__init__(**kw)
        self._arm_socks = {}   # {"left": sock, "right": sock}

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

    # -- 核心: run() --

    def execute(self, **kwargs):
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
                hand: 手势输入（预设名/数组/字典）
                arm: hand 目标臂，"left" 或 "right"（默认 "left"）
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

            # 播放指定臂的夹爪动作；未指定 arm 时保持左臂兼容默认值。
            hand = kwargs.get("hand")
            if hand is not None:
                results["hand"] = self.play_hand_gesture(
                    hand, arm=kwargs.get("arm", "left")
                )

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
                    results["pose"] = self.play_open_drawer(
                        speed=_resolve_drawer_speed(kwargs.get("speed"))
                    )
                elif name == "close_drawer":
                    results["pose"] = self.play_close_drawer(
                        speed=_resolve_drawer_speed(kwargs.get("speed"))
                    )
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
            speed = _resolve_drawer_speed(kwargs.get("speed"))
            ok = self.play_open_drawer(speed=speed)
            return {"success": ok, "info": "开抽屉" if ok else "开抽屉失败"}

        elif command == "close_drawer":
            arm = kwargs.get("arm", "right")
            if arm != "right":
                msg = f"动作 'close_drawer' 只能由右臂执行（请求的是 {arm}臂）"
                cprint(f"[pose_execute] 路由失败: {msg}", "red")
                return {"success": False, "info": msg}
            speed = _resolve_drawer_speed(kwargs.get("speed"))
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

        # 受控动作（轨迹回放，统一走 real/sim 执行器）
        if name in ACTION_ARM_RESTRICTIONS:
            drawer_speed = _resolve_drawer_speed(speed)
            if name == "open_drawer":
                return self.play_open_drawer(speed=drawer_speed)
            if name == "close_drawer":
                return self.play_close_drawer(speed=drawer_speed)

        poses = _load_poses(arm)
        pose_data = poses[name]
        joint_angles = pose_data["joint_angles_deg"]

        if self.config.sim_mode:
            return self._play_pose_sim(arm, name, joint_angles, speed)

        sock = self._get_arm_sock(arm)
        if sock is None:
            cprint(f"[pose_execute] [{arm}] 无可用连接", "red")
            return False

        try:
            resp = _send_arm_command(sock, joint_angles, arm=arm, speed=speed, block=block)
            cprint(f"[pose_execute] [{arm}] 执行位姿 {name}: {resp}", "green")
            return resp.get("value", False)
        except Exception as e:
            cprint(f"[pose_execute] [{arm}] 执行位姿失败: {e}", "red")
            self._arm_socks[arm] = None
            return False

    def _play_pose_sim(self, arm, name, joint_angles, speed=50):
        """sim 模式：单一位姿经 SimArmClient 下发到 PyBullet SimServer (:8031)。"""
        from core.sim_arm import SimArmClient
        host = self.config.shared.get("host", "127.0.0.1")
        client = SimArmClient(host, 8031, side=arm)
        if not client.connect():
            cprint(f"[pose_execute] (sim) [{arm}] 连接 SimServer 失败", "red")
            return False
        pose_dict = {f"J{i + 1}": float(v) for i, v in enumerate(joint_angles)}
        client.move_to_named_pose(pose_dict, speed=speed)
        cprint(f"[pose_execute] (sim) [{arm}] 执行位姿 {name}", "green")
        return True

    # -- 并行执行 --

    def play_parallel(self, actions):
        """
        并行执行多个动作

        Args:
            actions: 动作列表，每项是一个 dict，支持:
                - {"name": "pose_name", "arm": "left", "speed": 30}
                - {"hand": "open", "arm": "left"}
                - {"hand": "open", "arm": "right"}
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
                results[idx] = self.play_hand_gesture(
                    hand, arm=action.get("arm", "left")
                )

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

    # -- 双臂夹爪 --

    def play_hand_gesture(self, hand_input, arm="left"):
        """
        执行指定臂夹爪手势。

        仅支持 "open" / "close" 两种语义（Robotiq 85 夹爪），其他预设映射到 close。
        ``arm`` 默认为 ``left``，以兼容已有命令和旧动作序列。
        """
        return self._play_gripper(hand_input, arm)

    def _play_gripper(self, hand_input, arm="left"):
        """把手势映射为统一的双指夹爪 open/close 指令。"""
        if arm not in ("left", "right"):
            cprint(f"[pose_execute] 不支持的夹爪臂: {arm}", "red")
            return False

        force = None
        speed = None
        if isinstance(hand_input, str):
            key = hand_input.lower()
            if key in GRIPPER_PRESET_MAP:
                cmd_values = list(GRIPPER_PRESET_MAP[key])
            else:
                cprint(
                    f"[pose_execute] {arm}臂为夹爪，手势 '{hand_input}' 不支持，"
                    f"回落到 close",
                    "yellow",
                )
                cmd_values = [0, 0]
        elif isinstance(hand_input, dict):
            key = str(hand_input.get("action", "close")).lower()
            if key not in GRIPPER_PRESET_MAP:
                cprint(
                    f"[pose_execute] 夹爪动作 '{key}' 不支持，仅支持 open/close",
                    "red",
                )
                return False
            cmd_values = list(GRIPPER_PRESET_MAP[key])
            try:
                if hand_input.get("force") is not None:
                    force = int(hand_input["force"])
                if hand_input.get("speed") is not None:
                    speed = int(hand_input["speed"])
            except (TypeError, ValueError):
                cprint("[pose_execute] 夹爪 force/speed 必须是整数", "red")
                return False
            if force is not None and not 0 <= force <= 255:
                cprint("[pose_execute] 夹爪 force 必须在 0 到 255 之间", "red")
                return False
            if speed is not None and not 0 <= speed <= 255:
                cprint("[pose_execute] 夹爪 speed 必须在 0 到 255 之间", "red")
                return False
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

        try:
            # All real/sim routing, ports and namespaces are owned by the
            # common Skill.gripper_for(side) factory.
            gripper = self.gripper_for(arm)
            if cmd_values[0] > 500:
                resp = gripper.open(force=force, speed=speed)
            else:
                resp = gripper.close(force=force, speed=speed)
            label = hand_input if isinstance(hand_input, str) else cmd_values
            cprint(f"[pose_execute] 执行{arm}臂夹爪 {label}: {resp}", "green")
            return resp.get("value", False) in (True, [1000, 1000], [0, 0])
        except Exception as e:
            cprint(f"[pose_execute] {arm}臂夹爪指令失败: {e}", "red")
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
            - "hand": 手势 + 可选 "arm": "left"/"right"
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

            # 指定臂夹爪（先执行手）
            hand_gesture = step.get("hand")
            if hand_gesture is not None:
                if not self.play_hand_gesture(
                    hand_gesture, arm=step.get("arm", "left")
                ):
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

    # -- 轨迹回放（开关抽屉，统一委托给 real/sim 执行器） --

    def _play_trajectory(self, name, speed=DRAWER_TRAJECTORY_SPEED):
        """回放录制的轨迹（臂 + 夹爪）。

        Args:
            name: 轨迹名（不含路径和扩展名）
            speed: 回放速度倍率

        Returns:
            bool: 是否回放成功
        """
        from core.drawer_executor import create_drawer_executor
        executor = create_drawer_executor(self.config)
        return executor.play(name, speed=speed)

    def play_open_drawer(self, speed=DRAWER_TRAJECTORY_SPEED):
        """开抽屉：home → 抓把手 → 拉开 → 松手 → 回 home。

        Args:
            speed: 回放速度倍率 (默认 0.5x)

        Returns:
            bool: 是否成功
        """
        return self._play_trajectory("open_drawer", speed=speed)

    def play_close_drawer(self, speed=DRAWER_TRAJECTORY_SPEED):
        """关抽屉：home → 推关 → 回 home（不抓把手）。

        Args:
            speed: 回放速度倍率 (默认 0.5x)

        Returns:
            bool: 是否成功
        """
        return self._play_trajectory("close_drawer", speed=speed)
