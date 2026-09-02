#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位姿与轨迹录制工具
==================
开发阶段使用的独立工具脚本，用于录制机械臂位姿和连续运动轨迹。

位姿录制：
  直接连接机械臂硬件（Robotic_Arm SDK）读取当前关节状态。

轨迹录制：
  使用 SDK 拖拽示教模式 + 后台高频采样，录制连续运动轨迹。
  回放通过 CAN-FD 流式发送关节角实现速度可调。

录制结果:
  - 位姿保存在 recorded_poses/{left,right}.json
  - 轨迹保存在 recorded_trajectories/{left,right}/<name>.json

机械臂映射:
    left  → 192.168.1.19 (灵巧手)
    right → 192.168.1.18 (夹爪)

使用方式:

    # 位姿录制
    python3 tools/pose_record.py record --name grasp1 --arm left
    python3 tools/pose_record.py record -i --arm right
    python3 tools/pose_record.py list
    python3 tools/pose_record.py delete --name grasp1 --arm left

    # 轨迹录制与回放
    python3 tools/pose_record.py traj-record --name pick_orange --arm left
    python3 tools/pose_record.py traj-record --name wave --arm right --desc "挥手" --rate 50
    python3 tools/pose_record.py traj-play --name pick_orange --arm left
    python3 tools/pose_record.py traj-play --name wave --arm right --speed 0.5
    python3 tools/pose_record.py traj-list
    python3 tools/pose_record.py traj-info --name pick_orange --arm left
    python3 tools/pose_record.py traj-delete --name pick_orange --arm left
"""

import os
import sys
import json
import math
import socket
import time
import argparse
import threading
from datetime import datetime
from termcolor import cprint

# 机械臂接口 -- 直接连接硬件以读取当前关节状态
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POSE_DIR = os.path.join(PROJECT_ROOT, "recorded_poses")
TRAJ_DIR = os.path.join(PROJECT_ROOT, "recorded_trajectories")

POSE_FILES = {
    "left": os.path.join(POSE_DIR, "left.json"),
    "right": os.path.join(POSE_DIR, "right.json"),
}

TRAJ_DIRS = {
    "left": os.path.join(TRAJ_DIR, "left"),
    "right": os.path.join(TRAJ_DIR, "right"),
}

ARM_CONFIGS = {
    "left": {"ip": "192.168.1.19", "label": "左臂（灵巧手）"},
    "right": {"ip": "192.168.1.18", "label": "右臂（夹爪）"},
}
ARM_PORT = 8080
GRIPPER_PORTS = {"left": 8002, "right": 8001}
GRIPPER_SRCS = {
    "left": "/left_gripper/movement_control",
    "right": "/right_gripper/movement_control",
}
GRIPPER_COMMANDS = {"open": [1000, 1000], "close": [0, 0]}

# RealMan controller arm-error codes surfaced by rm_get_current_arm_state().
# Keep this informational only: a non-zero code must still block motion.
ARM_ERROR_DESCRIPTIONS = {
    14: "机械臂碰撞",
    19: "自碰撞",
    20: "电子围栏碰撞",
    21: "关节超出软限位",
}

# Replay safety parameters. The JSON trajectory is the only replay source;
# controller-side native trajectories may belong to an earlier recording.
START_MOVE_SPEED = 20
START_POSITION_TOLERANCE_DEG = 1.0
FINAL_POSITION_TOLERANCE_DEG = 1.5
STATE_POLL_INTERVAL_S = 0.05
HEALTH_CHECK_INTERVAL_S = 0.2
START_SETTLE_TIME_S = 2.0


# ---------------------------------------------------------------------------
# 共享工具
# ---------------------------------------------------------------------------

def _connect_arm(arm_side="left"):
    """
    连接机械臂硬件

    Returns:
        tuple: (arm, arm_handle) 或 (None, None)
    """
    arm_cfg = ARM_CONFIGS[arm_side]
    arm_ip = arm_cfg["ip"]

    try:
        arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
        arm_handle = arm.rm_create_robot_arm(arm_ip, ARM_PORT, level=3)
        if arm_handle.id == -1:
            cprint(f"[pose_record] 无法连接{arm_cfg['label']} {arm_ip}:{ARM_PORT}", "red")
            return None, None
        cprint(f"[pose_record] 已连接{arm_cfg['label']}, ID: {arm_handle.id}", "green")
        return arm, arm_handle
    except Exception as e:
        cprint(f"[pose_record] 连接机械臂失败: {e}", "red")
        return None, None


# ---------------------------------------------------------------------------
# 位姿: 加载/保存
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


# ---------------------------------------------------------------------------
# 位姿: 录制
# ---------------------------------------------------------------------------

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

    arm, arm_handle = _connect_arm(arm_side)
    if arm is None:
        return False

    try:
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

        poses = _load_poses(pose_file)
        poses[name] = pose_data
        _save_poses(poses, pose_file)

        cprint(f"\n[pose_record] 已录制位姿: {name}", "cyan")
        _print_pose(pose_data)
        return True
    finally:
        pass


def interactive_record(arm_side="left"):
    """交互式录制模式"""
    os.makedirs(POSE_DIR, exist_ok=True)
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


# ---------------------------------------------------------------------------
# 位姿: 管理
# ---------------------------------------------------------------------------

def list_poses(arm_side=None):
    """列出已录制的位姿"""
    sides = [arm_side] if arm_side else ["left", "right"]

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
# 轨迹: 加载/保存
# ---------------------------------------------------------------------------

def _load_trajectory(name, arm_side):
    """加载已保存的轨迹"""
    traj_file = os.path.join(TRAJ_DIRS[arm_side], f"{name}.json")
    if not os.path.exists(traj_file):
        return None
    with open(traj_file, "r") as f:
        return json.load(f)


def _save_trajectory(traj_data, arm_side):
    """保存轨迹到文件"""
    os.makedirs(TRAJ_DIRS[arm_side], exist_ok=True)
    traj_file = os.path.join(TRAJ_DIRS[arm_side], f"{traj_data['name']}.json")
    with open(traj_file, "w") as f:
        json.dump(traj_data, f, indent=2)
    cprint(f"[traj] 轨迹已保存到 {traj_file}", "green")


def _list_trajectory_files(arm_side=None):
    """列出目录中的轨迹文件名（不含 .json 后缀）"""
    sides = [arm_side] if arm_side else ["left", "right"]
    result = {}
    for side in sides:
        traj_dir = TRAJ_DIRS[side]
        if os.path.isdir(traj_dir):
            names = [f[:-5] for f in os.listdir(traj_dir) if f.endswith(".json")]
        else:
            names = []
        result[side] = sorted(names)
    return result


# ---------------------------------------------------------------------------
# 轨迹: 录制
# ---------------------------------------------------------------------------

def _poll_joints_worker(arm, waypoints, stop_event, interval_s):
    """
    后台线程：高频轮询关节角度

    Args:
        arm: RoboticArm 实例
        waypoints: 共享列表，每项为 [elapsed_ms, J1..J7]
        stop_event: 停止信号
        interval_s: 轮询间隔（秒）
    """
    start_time = time.monotonic()
    consecutive_failures = 0

    while not stop_event.is_set():
        try:
            tag, joints = arm.rm_get_joint_degree()
            if tag == 0:
                elapsed_ms = (time.monotonic() - start_time) * 1000
                waypoints.append([round(elapsed_ms, 1)] + [round(j, 3) for j in joints])
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                if consecutive_failures >= 5:
                    cprint("[traj] 关节读取连续失败 5 次，停止录制", "red")
                    stop_event.set()
                    return
        except Exception as e:
            consecutive_failures += 1
            if consecutive_failures >= 5:
                cprint(f"[traj] 轮询异常: {e}", "red")
                stop_event.set()
                return

        stop_event.wait(interval_s)


def traj_record(name, description="", arm_side="left", rate_hz=50):
    """
    录制连续轨迹（拖拽示教模式 + 高频采样）

    Args:
        name: 轨迹名称
        description: 轨迹描述
        arm_side: "left" 或 "right"
        rate_hz: 采样频率 Hz (10-100)

    Returns:
        bool: 是否录制成功
    """
    rate_hz = max(10, min(100, rate_hz))
    interval_s = 1.0 / rate_hz

    arm, arm_handle = _connect_arm(arm_side)
    if arm is None:
        return False

    label = ARM_CONFIGS[arm_side]["label"]
    waypoints = []
    stop_event = threading.Event()

    try:
        # 启动拖拽示教（带轨迹录制）
        cprint(f"\n[traj] 启动 {label} 拖拽示教模式...", "cyan")
        tag = arm.rm_start_drag_teach(trajectory_record=1)
        if tag != 0:
            cprint(f"[traj] 启动拖拽示教失败，返回码: {tag}", "red")
            return False

        # 启动后台采样线程
        poll_thread = threading.Thread(
            target=_poll_joints_worker,
            args=(arm, waypoints, stop_event, interval_s),
            daemon=True,
        )
        poll_thread.start()

        cprint(f"[traj] 正在录制（{rate_hz}Hz）... 拖动机械臂到目标位置", "green")
        cprint("[traj] 按 Enter 停止录制", "yellow")
        input()

        # 停止拖拽示教
        tag = arm.rm_stop_drag_teach()
        if tag != 0:
            cprint(f"[traj] 停止拖拽示教返回码: {tag}（非零，可能未录制成功）", "yellow")

        # 停止采样线程
        stop_event.set()
        poll_thread.join(timeout=2.0)

        if not waypoints:
            cprint("[traj] 未录制到任何数据点", "red")
            return False

        # 构建轨迹数据
        traj_data = {
            "name": name,
            "description": description,
            "arm": arm_side,
            "timestamp": datetime.now().isoformat(),
            "recording_rate_hz": rate_hz,
            "num_points": len(waypoints),
            "duration_ms": waypoints[-1][0],
            "start_joint_deg": waypoints[0][1:],
            "end_joint_deg": waypoints[-1][1:],
            "waypoints": waypoints,
        }

        _save_trajectory(traj_data, arm_side)

        cprint(f"\n[traj] 录制完成!", "cyan")
        cprint(f"  名称: {name}", "cyan")
        cprint(f"  时长: {waypoints[-1][0] / 1000:.1f}s", "cyan")
        cprint(f"  采样点: {len(waypoints)}", "cyan")
        cprint(f"  起始关节: {[f'{j:.1f}' for j in waypoints[0][1:]]}", "cyan")
        cprint(f"  结束关节: {[f'{j:.1f}' for j in waypoints[-1][1:]]}", "cyan")
        return True

    except KeyboardInterrupt:
        cprint("\n[traj] 用户中断录制，保存已录制数据...", "yellow")
        stop_event.set()
        try:
            arm.rm_stop_drag_teach()
        except Exception:
            pass

        if waypoints:
            traj_data = {
                "name": name,
                "description": description + " (中断)",
                "arm": arm_side,
                "timestamp": datetime.now().isoformat(),
                "recording_rate_hz": rate_hz,
                "num_points": len(waypoints),
                "duration_ms": waypoints[-1][0],
                "start_joint_deg": waypoints[0][1:],
                "end_joint_deg": waypoints[-1][1:],
                "waypoints": waypoints,
            }
            _save_trajectory(traj_data, arm_side)
            cprint(f"[traj] 已保存 {len(waypoints)} 个采样点", "green")
        return len(waypoints) > 0


# ---------------------------------------------------------------------------
# 轨迹: 回放
# ---------------------------------------------------------------------------

def _move_to_start(arm, start_joints, timeout_s=30):
    """
    计划运动到轨迹起点（阻塞等待收敛）

    Args:
        arm: RoboticArm 实例
        start_joints: 目标关节角度列表（度）
        timeout_s: 收敛超时时间

    Returns:
        bool: 是否成功到达
    """
    if len(start_joints) != 7 or not all(
        isinstance(j, (int, float)) and math.isfinite(float(j))
        for j in start_joints
    ):
        cprint("[traj] 起点关节数据无效，拒绝运动", "red")
        return False

    def read_joints():
        try:
            tag, joints = arm.rm_get_joint_degree()
            if tag != 0 or len(joints) != 7:
                return None
            if not all(math.isfinite(float(j)) for j in joints):
                return None
            return [float(j) for j in joints]
        except Exception:
            return None

    def at_target(joints, tolerance):
        return joints is not None and all(
            abs(current - target) <= tolerance
            for current, target in zip(joints, start_joints)
        )

    current = read_joints()
    if current is None:
        cprint("[traj] 无法读取当前关节状态，拒绝运动到起点", "red")
        return False
    healthy, reason = _check_arm_health(arm)
    if not healthy:
        cprint(f"[traj] 起点运动前安全状态检查失败: {reason}", "red")
        return False
    if at_target(current, START_POSITION_TOLERANCE_DEG):
        cprint("[traj] 当前已在轨迹起点附近", "green")
        return True

    cprint("[traj] 正在运动到轨迹起点（非阻塞，随后轮询确认）...", "cyan")
    try:
        # Some RM controllers return 1 from the multi-threaded block=1 path
        # after printing `receive_state: false`. Send non-blocking and verify
        # arrival from joint feedback instead of trusting that wait path.
        tag = arm.rm_movej(
            joint=list(start_joints), v=START_MOVE_SPEED,
            r=0, connect=0, block=0
        )
    except Exception as exc:
        cprint(f"[traj] 发送起点运动失败: {type(exc).__name__}: {exc}", "red")
        return False

    if tag != 0:
        # On this controller/SDK combination rm_movej may return 1 with
        # `receive_state: false` and leave the arm stationary.  Do not send
        # the first recorded waypoint as a jump.  Halt any possible pending
        # command, refresh feedback, and approach the start with small
        # CAN-FD joint increments instead.
        cprint(
            f"[traj] rm_movej 起点运动返回 {tag}，切换为小步长 CAN-FD 到起点",
            "yellow",
        )
        _safe_slow_stop(arm)
        time.sleep(0.1)
        current = read_joints()
        if current is None:
            cprint("[traj] 无法刷新当前位置，拒绝起点插值", "red")
            return False

        max_delta = max(abs(current[i] - start_joints[i]) for i in range(7))
        if max_delta <= START_POSITION_TOLERANCE_DEG:
            return True
        duration_s = max(3.0, min(15.0, max_delta / 10.0))
        step_s = 0.05
        steps = max(2, int(math.ceil(duration_s / step_s)))
        cprint(
            f"[traj] 起点插值: {max_delta:.1f}° 最大关节差，"
            f"{steps} 步、约 {steps * step_s:.1f}s",
            "cyan",
        )
        motion_started = False
        for step in range(1, steps + 1):
            healthy, reason = _check_arm_health(arm)
            if not healthy:
                cprint(f"[traj] 起点插值安全状态检查失败: {reason}", "red")
                _safe_slow_stop(arm)
                return False
            ratio = step / steps
            waypoint = [
                current[i] + (start_joints[i] - current[i]) * ratio
                for i in range(7)
            ]
            tag = arm.rm_movej_canfd(joint=waypoint, follow=False, expand=0)
            motion_started = True
            if tag != 0:
                cprint(f"[traj] 起点插值 CAN-FD 失败，返回码: {tag}", "red")
                _safe_slow_stop(arm)
                return False
            time.sleep(step_s)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            healthy, reason = _check_arm_health(arm)
            if not healthy:
                cprint(f"[traj] 起点插值后安全状态检查失败: {reason}", "red")
                _safe_slow_stop(arm)
                return False
            if at_target(read_joints(), START_POSITION_TOLERANCE_DEG):
                cprint("[traj] 已通过 CAN-FD 插值到达轨迹起点", "green")
                return True
            time.sleep(STATE_POLL_INTERVAL_S)
        cprint("[traj] CAN-FD 插值到达起点超时，执行缓停", "red")
        _safe_slow_stop(arm)
        return False

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        healthy, reason = _check_arm_health(arm)
        if not healthy:
            cprint(f"[traj] 到起点过程中安全状态检查失败: {reason}", "red")
            _safe_slow_stop(arm)
            return False
        current = read_joints()
        if at_target(current, START_POSITION_TOLERANCE_DEG):
            cprint("[traj] 已到达轨迹起点", "green")
            return True
        time.sleep(STATE_POLL_INTERVAL_S)

    cprint("[traj] 到达轨迹起点超时，执行缓停", "red")
    _safe_slow_stop(arm)
    return False


def _validate_waypoints(waypoints):
    """Validate and normalize [elapsed_ms, J1..J7] trajectory points."""
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError("轨迹至少需要 2 个采样点")

    normalized = []
    previous_t = None
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, list) or len(waypoint) < 8:
            raise ValueError(f"第 {index + 1} 个轨迹点不是 [时间 + 7轴] 格式")
        timestamp = float(waypoint[0])
        joints = [float(value) for value in waypoint[1:8]]
        if not math.isfinite(timestamp) or not all(math.isfinite(j) for j in joints):
            raise ValueError(f"第 {index + 1} 个轨迹点包含非有限数值")
        if previous_t is not None and timestamp <= previous_t:
            raise ValueError(f"轨迹时间戳必须严格递增（第 {index + 1} 点）")
        previous_t = timestamp
        normalized.append((timestamp, joints))
    return normalized


def _safe_slow_stop(arm):
    """Best-effort slow stop for an already-started motion."""
    try:
        tag = arm.rm_set_arm_slow_stop()
        if tag != 0:
            cprint(f"[traj] 缓停返回码: {tag}", "yellow")
    except Exception as exc:
        cprint(f"[traj] 缓停异常: {type(exc).__name__}: {exc}", "red")


def _check_arm_health(arm):
    """Return (healthy, reason); unknown state is treated as unhealthy."""
    try:
        tag, state = arm.rm_get_current_arm_state()
        if tag != 0 or not isinstance(state, dict):
            return False, f"读取机械臂状态失败(tag={tag})"
        arm_err = state.get("arm_err", 0)
        sys_err = state.get("sys_err", 0)
        if arm_err or sys_err:
            arm_err_value = int(arm_err) if isinstance(arm_err, (int, float)) else arm_err
            description = ARM_ERROR_DESCRIPTIONS.get(arm_err_value, "未知机械臂错误")
            return False, (
                f"机械臂错误 arm_err={arm_err}, sys_err={sys_err}"
                f"（{description}；请在示教器清除错误并确认机械臂离开安全围栏边界）"
            )
        return True, ""
    except Exception as exc:
        return False, f"读取机械臂状态异常: {type(exc).__name__}: {exc}"


def _playback_canfd(arm, waypoints, speed_multiplier=1.0):
    """
    CAN-FD 流式回放轨迹

    Args:
        arm: RoboticArm 实例
        waypoints: 轨迹点列表，每项为 [elapsed_ms, J1..J7]
        speed_multiplier: 速度倍率

    Returns:
        bool: 是否回放完成
    """
    try:
        normalized = _validate_waypoints(waypoints)
    except (TypeError, ValueError) as exc:
        cprint(f"[traj] 轨迹校验失败: {exc}", "red")
        return False


    if not isinstance(speed_multiplier, (int, float)) or not math.isfinite(float(speed_multiplier)):
        cprint("[traj] 速度倍率无效", "red")
        return False
    speed_multiplier = float(speed_multiplier)
    if speed_multiplier <= 0:
        cprint("[traj] 速度倍率必须大于 0", "red")
        return False

    intervals_s = [
        (normalized[i][0] - normalized[i - 1][0]) / 1000.0 / speed_multiplier
        for i in range(1, len(normalized))
    ]
    if any(interval <= 0 for interval in intervals_s):
        cprint("[traj] 轨迹时间间隔无效", "red")
        return False

    effective_interval_s = sum(intervals_s) / len(intervals_s)
    effective_rate = 1.0 / effective_interval_s
    cprint(
        f"[traj] 回放参数: {speed_multiplier}x 速度, "
        f"平均间隔 {effective_interval_s * 1000:.1f}ms, "
        f"有效频率 {effective_rate:.0f}Hz",
        "cyan",
    )

    # follow=True 要求不超过 10ms 周期；更慢的轨迹使用普通模式。
    use_follow = effective_interval_s <= 0.01
    health_check_every = max(1, round(HEALTH_CHECK_INTERVAL_S / effective_interval_s))
    motion_started = False
    playback_started = time.monotonic()

    try:
        elapsed_schedule = [0.0]
        for interval_s in intervals_s:
            elapsed_schedule.append(elapsed_schedule[-1] + interval_s)

        for i, (_, joint) in enumerate(normalized):
            remaining = elapsed_schedule[i] - (time.monotonic() - playback_started)
            if remaining > 0:
                time.sleep(remaining)

            tag = arm.rm_movej_canfd(joint=joint, follow=use_follow, expand=0)
            motion_started = True
            if tag != 0:
                cprint(f"[traj] CAN-FD 发送失败 at 点 {i + 1}/{len(normalized)}: tag={tag}", "red")
                _safe_slow_stop(arm)
                return False

            if i % health_check_every == 0:
                healthy, reason = _check_arm_health(arm)
                if not healthy:
                    cprint(f"[traj] 安全状态检查失败: {reason}", "red")
                    _safe_slow_stop(arm)
                    return False

            if (i + 1) % max(1, len(normalized) // 10) == 0:
                cprint(
                    f"[traj] 进度: {i + 1}/{len(normalized)} "
                    f"({(i + 1) * 100 // len(normalized)}%)",
                    "yellow",
                )

        final_joints = normalized[-1][1]
        deadline = time.monotonic() + max(3.0, min(10.0, effective_interval_s * 20))
        while time.monotonic() < deadline:
            tag, current = arm.rm_get_joint_degree()
            if tag == 0 and len(current) == 7 and all(
                abs(float(current[j]) - final_joints[j]) <= FINAL_POSITION_TOLERANCE_DEG
                for j in range(7)
            ):
                cprint("[traj] 轨迹终点已确认", "green")
                return True
            time.sleep(STATE_POLL_INTERVAL_S)

        cprint("[traj] 轨迹指令已发完，但终点未确认，执行缓停", "red")
        _safe_slow_stop(arm)
        return False

    except KeyboardInterrupt:
        cprint(f"\n[traj] 用户中断回放 at 点 {i + 1}/{len(normalized)}", "yellow")
        if motion_started:
            _safe_slow_stop(arm)
        return False
    except Exception as exc:
        cprint(f"[traj] 回放异常: {type(exc).__name__}: {exc}", "red")
        if motion_started:
            _safe_slow_stop(arm)
        return False


def _execute_post_actions(traj_data):
    """Execute explicit actions that follow a successfully finished arm path."""
    actions = traj_data.get("post_actions", [])
    if not actions:
        return True
    if not isinstance(actions, list):
        cprint("[traj] post_actions 格式无效，拒绝执行", "red")
        return False

    for action in actions:
        if not isinstance(action, dict):
            cprint("[traj] post_action 项格式无效，拒绝执行", "red")
            return False
        if action.get("type") != "gripper":
            cprint(f"[traj] 不支持的 post_action 类型: {action.get('type')}", "red")
            return False
        side = action.get("side")
        command = action.get("action")
        if side not in GRIPPER_PORTS or command not in GRIPPER_COMMANDS:
            cprint(f"[traj] 无效的夹爪后置动作: {action}", "red")
            return False

        request = {
            "src": GRIPPER_SRCS[side],
            "type": "set",
            "cmd": GRIPPER_COMMANDS[command],
        }
        try:
            with socket.create_connection(
                ("127.0.0.1", GRIPPER_PORTS[side]), timeout=3.0
            ) as sock:
                sock.sendall(json.dumps(request).encode("utf-8"))
                response = json.loads(sock.recv(1024).decode("utf-8"))
            if response.get("value") is False:
                cprint(f"[traj] 夹爪后置动作失败: {response}", "red")
                return False
            cprint(f"[traj] post-action: {side} gripper -> {command}", "green")
        except Exception as exc:
            cprint(f"[traj] 夹爪后置动作异常: {exc}", "red")
            return False
    return True


def _load_home_joints(arm_side):
    """Load the configured home joint pose for an arm."""
    config_path = os.path.join(PROJECT_ROOT, "robot_config.json")
    with open(config_path, "r") as f:
        config = json.load(f)
    home = config.get("arms", {}).get(arm_side, {}).get("home")
    if not isinstance(home, dict):
        raise ValueError(f"配置中缺少 {arm_side} 臂 home 位姿")
    joints = [float(home[f"J{i}"]) for i in range(1, 8)]
    if not all(math.isfinite(joint) for joint in joints):
        raise ValueError(f"配置中的 {arm_side} 臂 home 位姿包含无效关节值")
    return joints


def traj_play(name, arm_side="left", speed=1.0):
    """
    回放录制的轨迹

    Args:
        name: 轨迹名称
        arm_side: "left" 或 "right"
        speed: 速度倍率 (正数)

    Returns:
        bool: 是否回放成功
    """
    speed = max(0.1, speed)

    # 加载轨迹
    traj_data = _load_trajectory(name, arm_side)
    if traj_data is None:
        traj_file = os.path.join(TRAJ_DIRS[arm_side], f"{name}.json")
        cprint(f"[traj] 未找到轨迹: {name} (期望: {traj_file})", "red")
        cprint("[traj] 使用 traj-list 查看可用轨迹", "yellow")
        return False

    waypoints = traj_data.get("waypoints")
    try:
        normalized = _validate_waypoints(waypoints)
    except (TypeError, ValueError) as exc:
        cprint(f"[traj] 轨迹校验失败: {exc}", "red")
        return False

    # 只使用 JSON 采样轨迹。*_sdk_backup.txt 可能来自控制器中残留的
    # 旧原生轨迹，不能作为本次录制的回放源。
    start_joints = normalized[0][1]
    label = ARM_CONFIGS[arm_side]["label"]
    cprint(f"\n[traj] 轨迹: {name} ({label})", "cyan")
    cprint(f"  时长: {traj_data['duration_ms'] / 1000:.1f}s", "cyan")
    cprint(f"  采样点: {len(waypoints)}", "cyan")
    cprint(f"  速度: {speed}x", "cyan")

    arm, arm_handle = _connect_arm(arm_side)
    if arm is None:
        return False

    try:
        # 运动到起点
        if not _move_to_start(arm, start_joints):
            return False
        # The RM controller can briefly report a transient arm error when
        # CAN-FD streaming starts immediately after the positioning move.
        # Let the confirmed start pose settle before the first waypoint.
        if START_SETTLE_TIME_S > 0:
            cprint(
                f"[traj] 起点已确认，等待 {START_SETTLE_TIME_S:.1f}s 稳定后开始回放",
                "cyan",
            )
            time.sleep(START_SETTLE_TIME_S)

        # 流式回放；只有轨迹终点确认后才执行夹爪等后置动作。
        if not _playback_canfd(arm, waypoints, speed):
            return False
        if not _execute_post_actions(traj_data):
            return False

        # A handover trajectory may explicitly bind a delayed return-home
        # motion after its release action.  Keep the delay after the gripper
        # command so the recipient has time to take the object.
        return_home_after_s = traj_data.get("return_home_after_s")
        if return_home_after_s is None:
            return True
        try:
            return_home_after_s = float(return_home_after_s)
        except (TypeError, ValueError):
            cprint("[traj] return_home_after_s 格式无效，拒绝回 home", "red")
            return False
        if not math.isfinite(return_home_after_s) or return_home_after_s < 0:
            cprint("[traj] return_home_after_s 必须是非负有限数值", "red")
            return False
        if return_home_after_s:
            cprint(
                f"[traj] 夹爪动作完成，等待 {return_home_after_s:.1f}s 后回 home",
                "cyan",
            )
            time.sleep(return_home_after_s)
        home_joints = _load_home_joints(arm_side)
        cprint(f"[traj] 开始移动 {label} 到 home", "cyan")
        if not _move_to_start(arm, home_joints):
            cprint(f"[traj] {label} 回 home 失败", "red")
            return False
        cprint(f"[traj] {label} 已回到 home", "green")
        return True

    except KeyboardInterrupt:
        cprint("\n[traj] 用户中断回放", "yellow")
        return False
    finally:
        try:
            arm.rm_delete_robot_arm()
        except Exception as exc:
            cprint(f"[traj] 关闭机械臂连接失败: {type(exc).__name__}: {exc}", "yellow")


# ---------------------------------------------------------------------------
# 轨迹: 管理
# ---------------------------------------------------------------------------

def traj_list(arm_side=None):
    """列出已录制的轨迹"""
    all_trajs = _list_trajectory_files(arm_side)
    sides = [arm_side] if arm_side else ["left", "right"]

    has_any = False
    for side in sides:
        names = all_trajs.get(side, [])
        label = ARM_CONFIGS[side]["label"]

        if not names:
            cprint(f"\n{label}: 没有已录制的轨迹", "yellow")
            continue

        has_any = True
        cprint(f"\n{label} ({len(names)} 个):", "cyan")
        print("=" * 60)
        for n in names:
            traj_data = _load_trajectory(n, side)
            if traj_data:
                desc = traj_data.get("description", "")
                dur = traj_data.get("duration_ms", 0) / 1000
                pts = traj_data.get("num_points", 0)
                ts = traj_data.get("timestamp", "N/A")
                print(f"  {n}: {dur:.1f}s, {pts}点{f' - {desc}' if desc else ''} ({ts})")
            else:
                print(f"  {n}: (加载失败)")
        print("=" * 60)

    return has_any


def traj_info(name, arm_side):
    """显示轨迹详情"""
    traj_data = _load_trajectory(name, arm_side)
    if traj_data is None:
        cprint(f"[traj] 未找到轨迹: {name}", "red")
        return False

    waypoints = traj_data["waypoints"]
    label = ARM_CONFIGS[arm_side]["label"]

    print("=" * 60)
    cprint(f"轨迹详情: {name} ({label})", "cyan")
    print("=" * 60)
    print(f"  描述: {traj_data.get('description', 'N/A')}")
    print(f"  时间: {traj_data.get('timestamp', 'N/A')}")
    print(f"  采样率: {traj_data.get('recording_rate_hz', 'N/A')} Hz")
    print(f"  采样点: {traj_data.get('num_points', 0)}")
    print(f"  时长: {traj_data.get('duration_ms', 0) / 1000:.2f} s")

    start_j = traj_data.get("start_joint_deg", [])
    end_j = traj_data.get("end_joint_deg", [])
    if start_j:
        print(f"\n  起始关节 (度): {[f'{j:.2f}' for j in start_j]}")
    if end_j:
        print(f"  结束关节 (度): {[f'{j:.2f}' for j in end_j]}")

    # 关节运动范围
    if len(waypoints) >= 2:
        import numpy as np
        joints = np.array([wp[1:] for wp in waypoints])
        ranges = joints.max(axis=0) - joints.min(axis=0)
        print(f"\n  各关节运动范围 (度):")
        for i, r in enumerate(ranges):
            print(f"    J{i + 1}: {r:.2f}")

        # 平均角速度
        total_dt = waypoints[-1][0] / 1000.0  # 秒
        if total_dt > 0:
            diffs = np.abs(np.diff(joints, axis=0))
            dts = np.diff([wp[0] / 1000.0 for wp in waypoints])
            avg_velocities = np.mean(diffs / dts[:, None], axis=0)
            peak_velocities = np.max(diffs / dts[:, None], axis=0)
            print(f"\n  平均角速度 (度/秒):")
            for i, v in enumerate(avg_velocities):
                print(f"    J{i + 1}: {v:.1f}")
            print(f"  峰值角速度 (度/秒):")
            for i, v in enumerate(peak_velocities):
                print(f"    J{i + 1}: {v:.1f}")

    print("=" * 60)
    return True


def traj_delete(name, arm_side):
    """删除轨迹"""
    traj_file = os.path.join(TRAJ_DIRS[arm_side], f"{name}.json")
    sdk_backup = os.path.join(TRAJ_DIRS[arm_side], f"{name}_sdk_backup.txt")

    if not os.path.exists(traj_file):
        cprint(
            f"[traj] {ARM_CONFIGS[arm_side]['label']}中未找到轨迹: {name}",
            "red",
        )
        return False

    os.remove(traj_file)
    cprint(f"[traj] 已删除: {traj_file}", "green")

    if os.path.exists(sdk_backup):
        os.remove(sdk_backup)
        cprint(f"[traj] 已删除 SDK 备份: {sdk_backup}", "green")

    return True


# ---------------------------------------------------------------------------
# 双臂: 位姿对录制
# ---------------------------------------------------------------------------

def record_pair(name, description=""):
    """
    交互式录制双臂位姿对，保存到 robot_config.json 的 dual_arm 字段

    Args:
        name: 位姿对名称（如 left_to_right_handover）
        description: 描述

    Returns:
        bool: 是否录制成功
    """
    config_path = os.path.join(PROJECT_ROOT, "robot_config.json")
    with open(config_path, "r") as f:
        config = json.load(f)

    dual_arm = config.setdefault("dual_arm", {})
    pair = dual_arm.setdefault(name, {})

    cprint(f"\n=== 双臂位姿对录制: {name} ===", "cyan")
    if description:
        cprint(f"描述: {description}", "cyan")

    poses = {}
    for arm_side in ["left", "right"]:
        label = ARM_CONFIGS[arm_side]["label"]
        print("\n" + "-" * 50)
        cprint(f"[pair] 请将 {label} 拖到目标位姿，然后按 Enter 录制", "yellow")
        input(f"[pair] {label} 准备好了，按 Enter...")

        arm, arm_handle = _connect_arm(arm_side)
        if arm is None:
            cprint(f"[pair] 连接 {label} 失败，中止", "red")
            return False

        tag, arm_state = arm.rm_get_current_arm_state()
        if tag != 0:
            tag2, joints = arm.rm_get_joint_degree()
            if tag2 != 0:
                cprint(f"[pair] 读取 {label} 关节角失败", "red")
                return False
        else:
            joints = arm_state["joint"]

        joint_dict = {f"J{i+1}": round(j, 3) for i, j in enumerate(joints)}
        poses[arm_side] = joint_dict

        cprint(f"[pair] {label} 已录制:", "green")
        for jn, jv in joint_dict.items():
            print(f"  {jn}: {jv}")

    # 保存到 config
    pair["left_pose"] = poses["left"]
    pair["right_pose"] = poses["right"]
    pair["description"] = description
    pair["timestamp"] = datetime.now().isoformat()

    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

    cprint(f"\n[pair] 位姿对 '{name}' 已保存到 robot_config.json", "green")
    cprint(f"[pair] 左臂: {[f'{v:.1f}' for v in poses['left'].values()]}", "cyan")
    cprint(f"[pair] 右臂: {[f'{v:.1f}' for v in poses['right'].values()]}", "cyan")
    return True


def list_pairs():
    """列出所有双臂位姿对"""
    config_path = os.path.join(PROJECT_ROOT, "robot_config.json")
    with open(config_path, "r") as f:
        config = json.load(f)

    dual_arm = config.get("dual_arm", {})
    if not dual_arm:
        cprint("[pair] 没有双臂位姿对", "yellow")
        return

    cprint(f"\n双臂位姿对 ({len(dual_arm)} 个):", "cyan")
    print("=" * 60)
    for name, data in dual_arm.items():
        desc = data.get("description", "")
        ts = data.get("timestamp", "N/A")
        left_ok = any(v != 0 for v in data.get("left_pose", {}).values())
        right_ok = any(v != 0 for v in data.get("right_pose", {}).values())
        status = "已录制" if (left_ok and right_ok) else "未完成"
        print(f"  {name}: {status} {f'- {desc}' if desc else ''} ({ts})")
    print("=" * 60)


def play_pair(name):
    """
    回放双臂位姿对（并行执行两臂运动）

    Args:
        name: 位姿对名称
    """
    from threading import Thread

    config_path = os.path.join(PROJECT_ROOT, "robot_config.json")
    with open(config_path, "r") as f:
        config = json.load(f)

    pair = config.get("dual_arm", {}).get(name)
    if pair is None:
        cprint(f"[pair] 未找到位姿对: {name}", "red")
        return False

    left_pose = pair.get("left_pose")
    right_pose = pair.get("right_pose")
    if not left_pose or not right_pose:
        cprint(f"[pair] 位姿对 '{name}' 数据不完整", "red")
        return False

    left_joints = [left_pose[f"J{i+1}"] for i in range(7)]
    right_joints = [right_pose[f"J{i+1}"] for i in range(7)]

    cprint(f"\n[pair] 回放位姿对: {name}", "cyan")

    results = [None, None]

    def _move_arm(idx, arm_side, joints):
        arm, _ = _connect_arm(arm_side)
        if arm is None:
            results[idx] = False
            return
        tag = arm.rm_movej(joint=joints, v=20, r=0, connect=0, block=1)
        results[idx] = (tag == 0)
        label = ARM_CONFIGS[arm_side]["label"]
        cprint(f"[pair] {label} 到达: {'成功' if tag == 0 else f'失败({tag})'}", "green" if tag == 0 else "red")

    t_left = Thread(target=_move_arm, args=(0, "left", left_joints))
    t_right = Thread(target=_move_arm, args=(1, "right", right_joints))

    t_left.start()
    t_right.start()
    t_left.join()
    t_right.join()

    all_ok = all(r for r in results if r is not None)
    cprint(f"[pair] 回放{'完成' if all_ok else '失败'}", "green" if all_ok else "red")
    return all_ok


# ---------------------------------------------------------------------------
# CLI 入口
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="位姿与轨迹录制工具 (开发阶段使用)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
机械臂映射:
  left  → 192.168.1.19 (灵巧手)
  right → 192.168.1.18 (夹爪)

位姿录制:
  python3 tools/pose_record.py record --name grasp1 --arm left
  python3 tools/pose_record.py record --name handover_pose --arm right --desc "交接位"
  python3 tools/pose_record.py record -i --arm left
  python3 tools/pose_record.py record -i --arm right
  python3 tools/pose_record.py list
  python3 tools/pose_record.py delete --name grasp1 --arm left

轨迹录制与回放:
  python3 tools/pose_record.py traj-record --name pick_orange --arm left
  python3 tools/pose_record.py traj-record --name wave --arm right --desc "挥手" --rate 50
  python3 tools/pose_record.py traj-play --name pick_orange --arm left
  python3 tools/pose_record.py traj-play --name wave --arm right --speed 0.5
  python3 tools/pose_record.py traj-play --name wave --arm right --speed 2.0
  python3 tools/pose_record.py traj-list
  python3 tools/pose_record.py traj-info --name pick_orange --arm left
  python3 tools/pose_record.py traj-delete --name pick_orange --arm left

双臂位姿对:
  python3 tools/pose_record.py record-pair --name left_to_right_handover --desc "左交右"
  python3 tools/pose_record.py list-pairs
  python3 tools/pose_record.py play-pair --name left_to_right_handover
        """,
    )

    subparsers = parser.add_subparsers(dest="command", help="命令")

    # --- 位姿命令 ---

    # record
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

    # list
    list_parser = subparsers.add_parser("list", help="列出已录制的位姿")
    list_parser.add_argument(
        "--arm", "-a", choices=["left", "right"],
        help="筛选指定臂（不指定则显示全部）"
    )

    # delete
    del_parser = subparsers.add_parser("delete", help="删除指定位姿")
    del_parser.add_argument("--name", "-n", required=True, help="位姿名称")
    del_parser.add_argument(
        "--arm", "-a", required=True, choices=["left", "right"],
        help="从哪条臂删除"
    )

    # --- 轨迹命令 ---

    # traj-record
    traj_rec_parser = subparsers.add_parser("traj-record", help="录制连续轨迹（拖拽示教）")
    traj_rec_parser.add_argument("--name", "-n", required=True, help="轨迹名称")
    traj_rec_parser.add_argument("--desc", "-d", default="", help="轨迹描述")
    traj_rec_parser.add_argument(
        "--arm", "-a", required=True, choices=["left", "right"],
        help="选择机械臂"
    )
    traj_rec_parser.add_argument(
        "--rate", "-r", type=int, default=50,
        help="采样频率 Hz (10-100, 默认50)"
    )

    # traj-play
    traj_play_parser = subparsers.add_parser("traj-play", help="回放轨迹")
    traj_play_parser.add_argument("--name", "-n", required=True, help="轨迹名称")
    traj_play_parser.add_argument(
        "--arm", "-a", required=True, choices=["left", "right"],
        help="选择机械臂"
    )
    traj_play_parser.add_argument(
        "--speed", "-s", type=float, default=1.0,
        help="速度倍率 (默认1.0, 如0.5=半速, 2.0=两倍速)"
    )

    # traj-list
    subparsers.add_parser("traj-list", help="列出已录制的轨迹")

    # traj-info
    traj_info_parser = subparsers.add_parser("traj-info", help="查看轨迹详情")
    traj_info_parser.add_argument("--name", "-n", required=True, help="轨迹名称")
    traj_info_parser.add_argument(
        "--arm", "-a", required=True, choices=["left", "right"],
        help="选择机械臂"
    )

    # traj-delete
    traj_del_parser = subparsers.add_parser("traj-delete", help="删除轨迹")
    traj_del_parser.add_argument("--name", "-n", required=True, help="轨迹名称")
    traj_del_parser.add_argument(
        "--arm", "-a", required=True, choices=["left", "right"],
        help="从哪条臂删除"
    )

    # --- 双臂位姿对命令 ---

    # record-pair
    pair_rec_parser = subparsers.add_parser("record-pair", help="录制双臂位姿对")
    pair_rec_parser.add_argument("--name", "-n", required=True, help="位姿对名称（如 left_to_right_handover）")
    pair_rec_parser.add_argument("--desc", "-d", default="", help="描述")

    # list-pairs
    subparsers.add_parser("list-pairs", help="列出双臂位姿对")

    # play-pair
    pair_play_parser = subparsers.add_parser("play-pair", help="回放双臂位姿对")
    pair_play_parser.add_argument("--name", "-n", required=True, help="位姿对名称")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    # --- 位姿命令分发 ---
    if args.command == "record":
        if args.interactive:
            interactive_record(arm_side=args.arm)
        elif args.name:
            record_pose(args.name, args.desc, arm_side=args.arm)
        else:
            cprint("错误: 请指定 --name 或使用 --interactive 模式", "red")

    elif args.command == "list":
        list_poses(arm_side=args.arm if hasattr(args, "arm") and args.arm else None)

    elif args.command == "delete":
        delete_pose(args.name, arm_side=args.arm)

    # --- 轨迹命令分发 ---
    elif args.command == "traj-record":
        traj_record(args.name, args.desc, arm_side=args.arm, rate_hz=args.rate)

    elif args.command == "traj-play":
        traj_play(args.name, arm_side=args.arm, speed=args.speed)

    elif args.command == "traj-list":
        traj_list()

    elif args.command == "traj-info":
        traj_info(args.name, arm_side=args.arm)

    elif args.command == "traj-delete":
        traj_delete(args.name, arm_side=args.arm)

    # --- 双臂位姿对命令分发 ---
    elif args.command == "record-pair":
        record_pair(args.name, args.desc)

    elif args.command == "list-pairs":
        list_pairs()

    elif args.command == "play-pair":
        play_pair(args.name)


if __name__ == "__main__":
    main()
