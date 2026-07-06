#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
右臂 + 夹爪 位姿与序列录制工具
================================
记录右臂关节角度（Robotic_Arm SDK 直连硬件）和夹爪状态（TCP :8001 或手动指定）。
支持单步位姿录制、交互式序列录制和高频轨迹录制（臂+爪同步采样）。

依赖: conda 环境 anygrasp（提供 Robotic_Arm SDK）

使用方式:

    # 单步录制（自动读取夹爪状态）
    python3 tools/record_sequence.py record --name home --desc "初始位姿"

    # 单步录制（手动指定夹爪状态，跳过 TCP 连接）
    python3 tools/record_sequence.py record --name grasp_handle --gripper closed

    # 交互式序列录制（逐步引导）
    python3 tools/record_sequence.py sequence --name drawer_cycle --desc "开关抽屉"

    # 高频轨迹录制（拖拽示教 + 臂爪同步采样）
    python3 tools/record_sequence.py traj-record --name drawer_cycle --rate 10

    # 轨迹回放
    python3 tools/record_sequence.py traj-play --name drawer_cycle

    # 查看
    python3 tools/record_sequence.py list
    python3 tools/record_sequence.py show --name drawer_cycle
    python3 tools/record_sequence.py traj-list
    python3 tools/record_sequence.py traj-info --name drawer_cycle

输出:
  - 单步位姿 -> recorded_poses/right.json（与 pose_record.py 兼容，额外包含 gripper 字段）
  - 序列     -> recorded_sequences/<name>.json
  - 轨迹     -> recorded_trajectories/right/<name>.json
"""

import json
import os
import socket
import sys
import threading
import time
from datetime import datetime

from termcolor import cprint

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

# ---------------------------------------------------------------------------
# 路径常量
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEQUENCE_DIR = os.path.join(PROJECT_ROOT, "recorded_sequences")
POSE_FILE = os.path.join(PROJECT_ROOT, "recorded_poses", "right.json")
TRAJ_DIR = os.path.join(PROJECT_ROOT, "recorded_trajectories", "right")

# ---------------------------------------------------------------------------
# 硬件常量
# ---------------------------------------------------------------------------
ARM_IP = "192.168.1.18"
ARM_SDK_PORT = 8080
GRIPPER_HOST = "127.0.0.1"
GRIPPER_PORT = 8001
GRIPPER_SRC = "/right_gripper/movement_control"

# 夹爪状态映射（与 core/gripper.py 一致: 1000=开, 0=闭）
GRIPPER_OPEN_VALUES = [1000, 1000]
GRIPPER_CLOSED_VALUES = [0, 0]


# ===================================================================
# 硬件连接
# ===================================================================

def _connect_arm():
    """连接右臂 SDK。返回 (arm, handle) 或 (None, None)。"""
    try:
        arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
        handle = arm.rm_create_robot_arm(ARM_IP, ARM_SDK_PORT, level=3)
        if handle.id == -1:
            cprint(f"[record] 无法连接右臂 {ARM_IP}:{ARM_SDK_PORT}", "red")
            return None, None
        cprint(f"[record] 已连接右臂, 句柄 ID: {handle.id}", "green")
        return arm, handle
    except Exception as e:
        cprint(f"[record] 连接右臂失败: {e}", "red")
        return None, None


def _get_arm_state(arm):
    """读取当前关节角度 (度) 和末端位姿。返回 (joints, end_pose) 或 (None, None)。"""
    tag, state = arm.rm_get_current_arm_state()
    if tag == 0:
        return state["joint"], state["pose"]

    tag2, joints = arm.rm_get_joint_degree()
    if tag2 == 0:
        return joints, None

    cprint("[record] 无法读取关节角度 (rm_get_current_arm_state 和 rm_get_joint_degree 均失败)", "red")
    return None, None


def _get_gripper_state():
    """通过 TCP 读取夹爪状态。返回 (values, info) 或 (None, error_string)。"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect((GRIPPER_HOST, GRIPPER_PORT))
        req = json.dumps({"src": GRIPPER_SRC, "type": "get"})
        sock.sendall(req.encode("utf-8"))
        resp = json.loads(sock.recv(1024).decode("utf-8"))
        sock.close()
        value = resp.get("value")
        info = resp.get("info", "")
        if isinstance(value, list) and len(value) == 2:
            return value, info
        return None, f"unexpected response: {resp}"
    except socket.timeout:
        return None, "timeout (gripper server 未运行?)"
    except ConnectionRefusedError:
        return None, "connection refused (端口 8001 未监听)"
    except Exception as e:
        return None, str(e)


# ===================================================================
# 夹爪状态辅助
# ===================================================================

def _gripper_label(values):
    """将夹爪值转为可读标签。"""
    if not values or not isinstance(values, list) or len(values) < 2:
        return "unknown"
    avg = (values[0] + values[1]) / 2.0
    if avg >= 800:
        return "open"
    elif avg <= 200:
        return "closed"
    else:
        return f"partial({avg:.0f})"


def _resolve_gripper():
    """交互式获取夹爪状态: 先尝试 TCP，失败则让用户手动选择。"""
    values, info = _get_gripper_state()
    if values is not None:
        label = _gripper_label(values)
        cprint(f"[record] 夹爪自动读取: {label}  values={values}", "cyan")
        return values, info

    cprint(f"[record] 无法自动读取夹爪状态: {info}", "yellow")
    cprint("[record] 请手动指定当前夹爪状态", "yellow")
    while True:
        choice = input("  夹爪状态 [o=open / c=closed / s=skip 不记录]: ").strip().lower()
        if choice in ("o", "open"):
            return GRIPPER_OPEN_VALUES, "manual: open"
        elif choice in ("c", "closed"):
            return GRIPPER_CLOSED_VALUES, "manual: closed"
        elif choice in ("s", "skip"):
            return None, "skipped"
        else:
            cprint("  请输入 o / c / s", "red")


# ===================================================================
# 位姿输出
# ===================================================================

def _print_pose(pose_data):
    """打印位姿详情。"""
    print("=" * 56)
    print(f"  名称: {pose_data['name']}")
    print(f"  描述: {pose_data.get('description', '-')}")
    print(f"  时间: {pose_data['timestamp']}")

    joints = pose_data["joint_angles_deg"]
    print("\n  关节角度 (度):")
    for i, j in enumerate(joints):
        print(f"    J{i + 1}: {j:.3f}")

    ep = pose_data.get("end_pose")
    if ep:
        print(f"\n  末端位姿:")
        print(f"    位置: x={ep[0]:.4f}, y={ep[1]:.4f}, z={ep[2]:.4f} (m)")
        print(f"    姿态: rx={ep[3]:.4f}, ry={ep[4]:.4f}, rz={ep[5]:.4f} (rad)")

    g = pose_data.get("gripper", {})
    if g:
        print(f"\n  夹爪: {g.get('state', '-')}  values={g.get('values', '-')}")
    print("=" * 56)


def _to_config_format(joints):
    """将关节角度列表转为 robot_config.json 的 J1-J7 格式。"""
    return {f"J{i + 1}": round(j, 3) for i, j in enumerate(joints)}


# ===================================================================
# 单步位姿录制
# ===================================================================

def record_pose(name, description="", gripper_manual=None):
    """录制当前右臂 + 夹爪位姿。

    Args:
        name: 位姿名称
        description: 描述
        gripper_manual: None=自动读取, "open"/"closed"=手动指定值

    Returns:
        dict 或 None
    """
    arm, handle = _connect_arm()
    if arm is None:
        return None

    try:
        joints, end_pose = _get_arm_state(arm)
        if joints is None:
            return None

        # 夹爪状态
        if gripper_manual == "open":
            gripper_values = GRIPPER_OPEN_VALUES
            gripper_info = "manual"
        elif gripper_manual == "closed":
            gripper_values = GRIPPER_CLOSED_VALUES
            gripper_info = "manual"
        else:
            gripper_values, gripper_info = _resolve_gripper()

        label = _gripper_label(gripper_values) if gripper_values else "unknown"

        pose_data = {
            "name": name,
            "description": description,
            "arm": "right",
            "timestamp": datetime.now().isoformat(),
            "joint_angles_deg": [round(j, 4) for j in joints],
            "end_pose": [round(v, 4) for v in end_pose] if end_pose else None,
            "gripper": {
                "state": label,
                "values": gripper_values,
                "info": gripper_info,
            },
        }

        # 保存到 recorded_poses/right.json（与 pose_record.py 共用，追加 gripper 字段）
        os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
        poses = {}
        if os.path.exists(POSE_FILE):
            with open(POSE_FILE, "r") as f:
                poses = json.load(f)
        poses[name] = pose_data
        with open(POSE_FILE, "w") as f:
            json.dump(poses, f, indent=2, ensure_ascii=False)

        cprint(f"\n[record] ✓ 位姿已保存: {name}  →  {POSE_FILE}", "green")
        _print_pose(pose_data)
        return pose_data

    finally:
        pass  # SDK 连接由调用方管理


# ===================================================================
# 交互式序列录制
# ===================================================================

def record_sequence(name, description=""):
    """交互式序列录制 —— 逐步引导用户录制每个动作位姿。

    每一步:
      1. 用户将机械臂拖到目标位姿，设置好夹爪状态
      2. 输入步骤名称
      3. 脚本录制当前臂 + 爪状态
      4. 重复直到用户输入 q

    序列保存到 recorded_sequences/<name>.json。
    """
    cprint(f"\n{'=' * 56}", "cyan")
    cprint(f"  序列录制模式: {name}", "cyan")
    cprint(f"{'=' * 56}", "cyan")
    if description:
        cprint(f"  描述: {description}", "cyan")

    print()
    cprint("📋 操作说明:", "yellow")
    cprint("  1. 将机械臂切换到示教模式", "yellow")
    cprint("  2. 手动将机械臂拖到目标位姿，调整夹爪状态", "yellow")
    cprint("  3. 输入步骤名称，脚本自动录制臂+爪当前状态", "yellow")
    cprint("  4. 重复步骤 2-3 直到完成所有步骤", "yellow")
    cprint("  5. 输入 'q' 结束录制并保存序列", "yellow")
    print()

    steps = []
    step_num = 1

    while True:
        print("-" * 40)
        cprint(f"  步骤 {step_num}", "cyan")
        step_name = input("  步骤名称 (如 approach_drawer, 输入 q 结束): ").strip()

        if step_name.lower() == "q":
            if steps:
                cprint(f"\n  已录制 {len(steps)} 步，正在保存...", "yellow")
            break
        if not step_name:
            cprint("  ⚠ 名称不能为空", "red")
            continue

        step_desc = input("  步骤描述 (可选): ").strip()

        # 检查是否手动指定夹爪
        gripper_choice = input("  夹爪 [回车=自动读取 / o=open / c=closed]: ").strip().lower()
        gripper_manual = None
        if gripper_choice in ("o", "open"):
            gripper_manual = "open"
        elif gripper_choice in ("c", "closed"):
            gripper_manual = "closed"

        input(f"\n  ⏳ 确保机械臂就位后，按 Enter 录制步骤 '{step_name}'...")

        pose = record_pose(step_name, step_desc, gripper_manual=gripper_manual)
        if pose is None:
            cprint(f"  ✗ 录制失败，请重试 (或输入 'q' 跳过)", "red")
            continue

        steps.append(pose)
        step_num += 1

    if not steps:
        cprint("\n[record] 未录制任何步骤，已取消", "yellow")
        return None

    # 保存序列
    seq_data = {
        "name": name,
        "description": description,
        "arm": "right",
        "timestamp": datetime.now().isoformat(),
        "num_steps": len(steps),
        "steps": [s["name"] for s in steps],
        "poses": {s["name"]: s for s in steps},
    }

    os.makedirs(SEQUENCE_DIR, exist_ok=True)
    seq_file = os.path.join(SEQUENCE_DIR, f"{name}.json")
    with open(seq_file, "w") as f:
        json.dump(seq_data, f, indent=2, ensure_ascii=False)

    # 汇总
    cprint(f"\n{'=' * 56}", "green")
    cprint(f"  ✓ 序列已保存: {seq_file}", "green")
    cprint(f"  共 {len(steps)} 步:", "green")
    for i, s in enumerate(steps):
        g = s.get("gripper", {})
        cprint(f"    {i + 1}. {s['name']:30s}  夹爪: {g.get('state', '-')}", "cyan")

    # 输出可直接复制到 robot_config.json 的格式
    cprint(f"\n{'─' * 56}", "yellow")
    cprint("  robot_config.json 格式 (可复制到 right arm 配置):", "yellow")
    for s in steps:
        js = _to_config_format(s["joint_angles_deg"])
        print(f'    "{s["name"]}": {json.dumps(js)},')
    cprint(f"{'─' * 56}", "yellow")

    return seq_data


# ===================================================================
# 管理命令
# ===================================================================

def list_sequences():
    """列出所有已录制的序列。"""
    if not os.path.isdir(SEQUENCE_DIR):
        cprint("[record] 没有已录制的序列", "yellow")
        return

    files = sorted(
        [f for f in os.listdir(SEQUENCE_DIR) if f.endswith(".json")]
    )
    if not files:
        cprint("[record] 没有已录制的序列", "yellow")
        return

    cprint(f"\n已录制的序列 ({len(files)} 个):", "cyan")
    print("=" * 65)
    for fn in files:
        with open(os.path.join(SEQUENCE_DIR, fn), "r") as f:
            seq = json.load(f)
        name = seq["name"]
        desc = seq.get("description", "-")
        n = seq.get("num_steps", len(seq.get("steps", [])))
        ts = seq.get("timestamp", "-")
        print(f"  {name:35s} {n} 步  {desc[:20]:20s}  {ts}")
    print("=" * 65)


def show_sequence(name):
    """显示序列详情。"""
    seq_file = os.path.join(SEQUENCE_DIR, f"{name}.json")
    if not os.path.exists(seq_file):
        cprint(f"[record] 序列不存在: {name}", "red")
        cprint(f"[record] 查找路径: {seq_file}", "red")
        return

    with open(seq_file, "r") as f:
        seq = json.load(f)

    cprint(f"\n=== 序列: {name} ===", "cyan")
    print(f"  描述:     {seq.get('description', '-')}")
    print(f"  时间:     {seq.get('timestamp', '-')}")
    print(f"  步骤数:   {seq.get('num_steps', 0)}")
    print(f"  步骤列表: {seq.get('steps', [])}")

    poses = seq.get("poses", {})
    for step_name in seq.get("steps", []):
        if step_name in poses:
            print(f"\n--- {step_name} ---")
            _print_pose(poses[step_name])


# ===================================================================
# 高频轨迹录制（臂 + 爪同步采样）
# ===================================================================

def _try_read_gripper():
    """快速读取夹爪状态，失败返回 None。"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.3)
        sock.connect((GRIPPER_HOST, GRIPPER_PORT))
        sock.sendall(json.dumps({"src": GRIPPER_SRC, "type": "get"}).encode())
        resp = json.loads(sock.recv(256).decode())
        sock.close()
        v = resp.get("value")
        if isinstance(v, list) and len(v) == 2:
            return v
        return None
    except Exception:
        return None


def _poll_arm_and_gripper(arm, waypoints, stop_event, interval_s, record_gripper):
    """后台线程：高频轮询关节角度 + 夹爪状态。

    waypoints 每项: [elapsed_ms, J1..J7] 或 [elapsed_ms, J1..J7, gripper_v1, gripper_v2]
    """
    start_time = time.monotonic()
    failures = 0
    gripper_failures = 0

    while not stop_event.is_set():
        try:
            tag, joints = arm.rm_get_joint_degree()
            if tag == 0:
                elapsed_ms = (time.monotonic() - start_time) * 1000
                entry = [round(elapsed_ms, 1)] + [round(j, 3) for j in joints]

                if record_gripper:
                    gv = _try_read_gripper()
                    if gv is not None:
                        entry += gv
                        gripper_failures = 0
                    else:
                        # 沿用上一次的值，或填 -1
                        if waypoints and len(waypoints[-1]) >= 10:
                            entry += waypoints[-1][8:10]
                        else:
                            entry += [-1, -1]
                        gripper_failures += 1
                        if gripper_failures > 50:
                            cprint("[traj] 夹爪连续读取失败 50 次，停止夹爪记录", "yellow")

                waypoints.append(entry)
                failures = 0
            else:
                failures += 1
                if failures >= 5:
                    cprint("[traj] 关节读取连续失败 5 次，停止录制", "red")
                    stop_event.set()
                    return
        except Exception as e:
            failures += 1
            if failures >= 5:
                cprint(f"[traj] 轮询异常: {e}", "red")
                stop_event.set()
                return

        stop_event.wait(interval_s)


def traj_record(name, description="", rate_hz=10, record_gripper=True, duration=0,
                gripper_script=None):
    """拖拽示教 + 高频采样，同步记录机械臂关节角和夹爪状态。

    采样率 rate_hz (Hz)，默认 10Hz（每 0.1 秒一次）。

    保存到 recorded_trajectories/right/<name>.json。
    """
    rate_hz = max(5, min(100, rate_hz))
    interval_s = 1.0 / rate_hz

    arm, handle = _connect_arm()
    if arm is None:
        return False

    waypoints = []
    stop_event = threading.Event()

    # 检查夹爪是否可用
    if record_gripper or gripper_script:
        gv = _try_read_gripper()
        if gv is None:
            if gripper_script:
                cprint("[traj] 夹爪服务器不可达，无法执行夹爪脚本", "red")
                return False
            cprint("[traj] 夹爪服务器不可达，仅记录关节角度", "yellow")
            record_gripper = False
        else:
            cprint(f"[traj] 夹爪服务器在线，将同步记录夹爪状态", "green")

    # 夹爪脚本线程
    gripper_script_stop = threading.Event()
    if gripper_script:
        cprint(f"[traj] 夹爪脚本: {gripper_script}", "cyan")

        def _run_gripper_script():
            start = time.monotonic()
            for delay_s, action in sorted(gripper_script):
                if gripper_script_stop.is_set():
                    return
                elapsed = time.monotonic() - start
                wait = delay_s - elapsed
                if wait > 0:
                    gripper_script_stop.wait(wait)
                if gripper_script_stop.is_set():
                    return
                cmd = GRIPPER_OPEN_VALUES if action == "open" else GRIPPER_CLOSED_VALUES
                cprint(f"\n[traj] 夹爪脚本 @ t≈{delay_s}s: {action}", "yellow")
                _send_gripper_cmd(cmd)

        script_thread = threading.Thread(target=_run_gripper_script, daemon=True)
        script_thread.start()

    try:
        cprint(f"\n[traj] 启动拖拽示教模式 (采样率 {rate_hz}Hz, 间隔 {interval_s*1000:.0f}ms)...", "cyan")
        tag = arm.rm_start_drag_teach(trajectory_record=1)
        if tag != 0:
            cprint(f"[traj] 启动拖拽示教失败，返回码: {tag}", "red")
            return False

        # 启动后台采样线程
        poll_thread = threading.Thread(
            target=_poll_arm_and_gripper,
            args=(arm, waypoints, stop_event, interval_s, record_gripper),
            daemon=True,
        )
        poll_thread.start()

        cols = ["elapsed_ms", "J1", "J2", "J3", "J4", "J5", "J6", "J7"]
        if record_gripper:
            cols += ["gripper_v1", "gripper_v2"]
        cprint(f"[traj] 列: {cols}", "cyan")
        if duration and duration > 0:
            # 固定时长模式
            cprint("[traj] 3 秒后开始录制...", "yellow")
            for i in range(3, 0, -1):
                cprint(f"[traj] {i}...", "yellow")
                time.sleep(1)
            cprint(f"[traj] ▶ 开始! 录制 {duration}s", "green")
            for remaining in range(int(duration), 0, -5):
                if stop_event.is_set():
                    break
                time.sleep(min(5, remaining))
                if not stop_event.is_set():
                    cprint(f"[traj] ...剩余 {max(0, remaining - 5)}s", "yellow")
        else:
            # 交互模式：Enter 开始，Enter 停止
            input("[traj] 按 Enter 开始录制...")
            cprint(f"[traj] ▶ 开始! 按 Enter 停止", "green")
            input()
            cprint("[traj] ⏹ 停止", "yellow")

        # 停止拖拽示教
        tag = arm.rm_stop_drag_teach()
        # 停止采样线程和夹爪脚本
        stop_event.set()
        gripper_script_stop.set()
        poll_thread.join(timeout=2.0)

        if not waypoints:
            cprint("[traj] 未录制到任何数据点", "red")
            return False

        # 构建轨迹数据
        gripper_cols = ["gripper_v1", "gripper_v2"] if record_gripper else []
        traj_data = {
            "name": name,
            "description": description,
            "arm": "right",
            "timestamp": datetime.now().isoformat(),
            "recording_rate_hz": rate_hz,
            "num_points": len(waypoints),
            "duration_ms": waypoints[-1][0],
            "columns": cols,
            "start_joint_deg": waypoints[0][1:8],
            "end_joint_deg": waypoints[-1][1:8],
            "recorded_gripper": record_gripper,
            "waypoints": waypoints,
        }
        if record_gripper and len(waypoints) > 0 and len(waypoints[0]) >= 10:
            traj_data["start_gripper"] = waypoints[0][8:10]
            traj_data["end_gripper"] = waypoints[-1][8:10]

        os.makedirs(TRAJ_DIR, exist_ok=True)
        traj_file = os.path.join(TRAJ_DIR, f"{name}.json")
        with open(traj_file, "w") as f:
            json.dump(traj_data, f, indent=2)

        cprint(f"\n[traj] ✓ 轨迹已保存: {traj_file}", "green")
        cprint(f"  名称: {name}", "cyan")
        cprint(f"  时长: {waypoints[-1][0] / 1000:.1f}s", "cyan")
        cprint(f"  采样点: {len(waypoints)}", "cyan")
        cprint(f"  夹爪记录: {'是' if record_gripper else '否'}", "cyan")
        cprint(f"  起始关节: {[f'{j:.1f}' for j in waypoints[0][1:8]]}", "cyan")
        cprint(f"  结束关节: {[f'{j:.1f}' for j in waypoints[-1][1:8]]}", "cyan")
        return True

    except KeyboardInterrupt:
        cprint("\n[traj] 用户中断，保存已录制数据...", "yellow")
        stop_event.set()
        gripper_script_stop.set()
        try:
            arm.rm_stop_drag_teach()
        except Exception:
            pass
        # 保存部分数据
        if waypoints:
            os.makedirs(TRAJ_DIR, exist_ok=True)
            traj_file = os.path.join(TRAJ_DIR, f"{name}.json")
            traj_data = {
                "name": name,
                "description": description + " (中断)",
                "arm": "right",
                "timestamp": datetime.now().isoformat(),
                "recording_rate_hz": rate_hz,
                "num_points": len(waypoints),
                "duration_ms": waypoints[-1][0],
                "columns": cols,
                "start_joint_deg": waypoints[0][1:8],
                "end_joint_deg": waypoints[-1][1:8],
                "recorded_gripper": record_gripper,
                "waypoints": waypoints,
            }
            with open(traj_file, "w") as f:
                json.dump(traj_data, f, indent=2)
            cprint(f"[traj] 已保存 {len(waypoints)} 个采样点", "green")
        return len(waypoints) > 0


def traj_list():
    """列出已录制的轨迹。"""
    if not os.path.isdir(TRAJ_DIR):
        cprint("[traj] 没有已录制的轨迹", "yellow")
        return

    files = sorted([f for f in os.listdir(TRAJ_DIR) if f.endswith(".json")])
    if not files:
        cprint("[traj] 没有已录制的轨迹", "yellow")
        return

    cprint(f"\n已录制的轨迹 ({len(files)} 个):", "cyan")
    print("=" * 70)
    for fn in files:
        fpath = os.path.join(TRAJ_DIR, fn)
        with open(fpath, "r") as f:
            t = json.load(f)
        name = t["name"]
        dur = t.get("duration_ms", 0) / 1000
        pts = t.get("num_points", 0)
        rate = t.get("recording_rate_hz", "?")
        has_g = "✓" if t.get("recorded_gripper") else "✗"
        ts = t.get("timestamp", "-")
        print(f"  {name:30s} {dur:5.1f}s  {pts:5d}点  {rate:3d}Hz  夹爪:{has_g}  {ts}")
    print("=" * 70)


def traj_info(name):
    """显示轨迹详情。"""
    traj_file = os.path.join(TRAJ_DIR, f"{name}.json")
    if not os.path.exists(traj_file):
        cprint(f"[traj] 轨迹不存在: {name}", "red")
        return

    with open(traj_file, "r") as f:
        t = json.load(f)

    print("=" * 60)
    cprint(f"轨迹详情: {name}", "cyan")
    print("=" * 60)
    print(f"  描述:       {t.get('description', '-')}")
    print(f"  时间:       {t.get('timestamp', '-')}")
    print(f"  采样率:     {t.get('recording_rate_hz', '?')} Hz")
    print(f"  采样点:     {t.get('num_points', 0)}")
    print(f"  时长:       {t.get('duration_ms', 0) / 1000:.2f} s")
    print(f"  记录夹爪:   {t.get('recorded_gripper', False)}")
    print(f"  列:         {t.get('columns', [])}")

    start_j = t.get("start_joint_deg", [])
    end_j = t.get("end_joint_deg", [])
    if start_j:
        print(f"\n  起始关节 (度): {[f'{j:.2f}' for j in start_j]}")
    if end_j:
        print(f"  结束关节 (度): {[f'{j:.2f}' for j in end_j]}")

    sg = t.get("start_gripper")
    eg = t.get("end_gripper")
    if sg:
        print(f"  起始夹爪:     {sg}")
    if eg:
        print(f"  结束夹爪:     {eg}")

    # 关节运动范围
    waypoints = t.get("waypoints", [])
    if len(waypoints) >= 2:
        joints = [[wp[i] for i in range(1, 8)] for wp in waypoints]
        import numpy as np
        jarr = np.array(joints)
        ranges = jarr.max(axis=0) - jarr.min(axis=0)
        print(f"\n  各关节运动范围 (度):")
        for i, r in enumerate(ranges):
            print(f"    J{i + 1}: {r:.2f}")

    print("=" * 60)


def traj_play(name, speed=1.0, gripper_enabled=True):
    """回放录制的轨迹（臂 + 爪）。

    使用 rm_movej_canfd 流式发送关节角，同步控制夹爪。
    """
    traj_file = os.path.join(TRAJ_DIR, f"{name}.json")
    if not os.path.exists(traj_file):
        cprint(f"[traj] 轨迹不存在: {name}", "red")
        return False

    with open(traj_file, "r") as f:
        t = json.load(f)

    waypoints = t["waypoints"]
    has_gripper = t.get("recorded_gripper", False) and gripper_enabled

    cprint(f"\n[traj] 回放轨迹: {name}", "cyan")
    cprint(f"  时长: {t['duration_ms'] / 1000:.1f}s", "cyan")
    cprint(f"  采样点: {len(waypoints)}", "cyan")
    cprint(f"  速度: {speed}x", "cyan")

    arm, handle = _connect_arm()
    if arm is None:
        return False

    # 运动到起点
    cprint("[traj] 运动到轨迹起点...", "cyan")
    start_joints = t["start_joint_deg"]
    tag = arm.rm_movej(joint=start_joints, v=20, r=0, connect=0, block=1)
    if tag != 0:
        cprint(f"[traj] 运动到起点失败 tag={tag}", "red")
        return False

    # 设置起始夹爪
    if has_gripper and len(waypoints[0]) >= 10:
        gv = waypoints[0][8:10]
        _send_gripper_cmd(gv)

    # 计算回放参数
    if len(waypoints) < 2:
        cprint("[traj] 轨迹点不足", "red")
        return False

    total_dt_ms = waypoints[-1][0] - waypoints[0][0]
    num_intervals = len(waypoints) - 1
    base_interval_s = (total_dt_ms / num_intervals) / 1000.0
    adjusted_interval_s = base_interval_s / speed
    adjusted_interval_s = max(0.005, adjusted_interval_s)

    cprint(f"[traj] 回放间隔: {adjusted_interval_s * 1000:.1f}ms", "cyan")

    try:
        for i, wp in enumerate(waypoints):
            joint = wp[1:8]
            tag = arm.rm_movej_canfd(joint=joint, follow=False, expand=0)

            # 夹爪控制（每 5 帧发一次，减少 TCP 负载）
            if has_gripper and len(wp) >= 10 and i % 5 == 0:
                gv = wp[8:10]
                _send_gripper_cmd(gv)

            if (i + 1) % max(1, len(waypoints) // 10) == 0:
                cprint(
                    f"[traj] 进度: {i + 1}/{len(waypoints)} "
                    f"({(i + 1) * 100 // len(waypoints)}%)", "yellow"
                )

            time.sleep(adjusted_interval_s)

        cprint("[traj] ✓ 回放完成", "green")
        return True

    except KeyboardInterrupt:
        cprint(f"\n[traj] 用户中断 at 点 {i + 1}/{len(waypoints)}", "yellow")
        return False


def traj_delete(name):
    """删除轨迹。"""
    traj_file = os.path.join(TRAJ_DIR, f"{name}.json")
    if not os.path.exists(traj_file):
        cprint(f"[traj] 轨迹不存在: {name}", "red")
        return False
    os.remove(traj_file)
    cprint(f"[traj] 已删除: {traj_file}", "green")
    return True


def _send_gripper_cmd(cmd_values):
    """发送夹爪指令，静默失败。"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        sock.connect((GRIPPER_HOST, GRIPPER_PORT))
        sock.sendall(json.dumps({
            "src": GRIPPER_SRC, "type": "set", "cmd": list(cmd_values),
        }).encode())
        sock.recv(256)
        sock.close()
    except Exception:
        pass


# ===================================================================
# CLI
# ===================================================================

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="右臂 + 夹爪 位姿与序列录制工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 单步录制
  python3 tools/record_sequence.py record --name home --desc "初始位姿"
  python3 tools/record_sequence.py record --name grasp_handle --gripper closed

  # 序列录制
  python3 tools/record_sequence.py sequence --name drawer_cycle --desc "开关抽屉"

  # 查看
  python3 tools/record_sequence.py list
  python3 tools/record_sequence.py show --name drawer_cycle
        """,
    )

    sub = parser.add_subparsers(dest="command", help="命令")

    # ---- record ----
    rec = sub.add_parser("record", help="录制单步位姿 (臂 + 爪)")
    rec.add_argument("--name", "-n", required=True, help="位姿名称")
    rec.add_argument("--desc", "-d", default="", help="描述")
    rec.add_argument(
        "--gripper", "-g", choices=["open", "closed"],
        help="手动指定夹爪状态 (不指定则自动读取 :8001)",
    )

    # ---- sequence ----
    seq = sub.add_parser("sequence", help="交互式序列录制")
    seq.add_argument("--name", "-n", required=True, help="序列名称")
    seq.add_argument("--desc", "-d", default="", help="序列描述")

    # ---- list ----
    sub.add_parser("list", help="列出已录制序列")

    # ---- show ----
    show = sub.add_parser("show", help="显示序列详情")
    show.add_argument("--name", "-n", required=True, help="序列名称")

    # ---- traj-record ----
    traj_rec = sub.add_parser("traj-record", help="高频轨迹录制 (拖拽示教, 臂+爪同步采样)")
    traj_rec.add_argument("--name", "-n", required=True, help="轨迹名称")
    traj_rec.add_argument("--desc", "-d", default="", help="描述")
    traj_rec.add_argument(
        "--rate", "-r", type=int, default=10,
        help="采样率 Hz (5-100, 默认 10Hz=0.1s/次)",
    )
    traj_rec.add_argument(
        "--duration", type=float, default=0,
        help="录制时长 (秒)，0=手动按 Enter 结束",
    )
    traj_rec.add_argument(
        "--no-gripper", action="store_true",
        help="不记录夹爪状态（仅关节角）",
    )
    traj_rec.add_argument(
        "--gripper-script",
        help="夹爪动作脚本: 逗号分隔的 t:action, 如 '0:open,8:close,20:open'",
    )

    # ---- traj-play ----
    traj_play_p = sub.add_parser("traj-play", help="回放轨迹")
    traj_play_p.add_argument("--name", "-n", required=True, help="轨迹名称")
    traj_play_p.add_argument(
        "--speed", "-s", type=float, default=1.0,
        help="速度倍率 (默认 1.0)",
    )
    traj_play_p.add_argument(
        "--no-gripper", action="store_true",
        help="回放时不控制夹爪",
    )

    # ---- traj-list ----
    sub.add_parser("traj-list", help="列出已录制轨迹")

    # ---- traj-info ----
    traj_info_p = sub.add_parser("traj-info", help="查看轨迹详情")
    traj_info_p.add_argument("--name", "-n", required=True, help="轨迹名称")

    # ---- traj-delete ----
    traj_del = sub.add_parser("traj-delete", help="删除轨迹")
    traj_del.add_argument("--name", "-n", required=True, help="轨迹名称")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    if args.command == "record":
        record_pose(args.name, args.desc, gripper_manual=args.gripper)

    elif args.command == "sequence":
        record_sequence(args.name, args.desc)

    elif args.command == "list":
        list_sequences()

    elif args.command == "show":
        show_sequence(args.name)

    elif args.command == "traj-record":
        # 解析夹爪脚本
        gripper_script = None
        if args.gripper_script:
            gripper_script = []
            for part in args.gripper_script.split(","):
                t_str, action = part.strip().split(":")
                gripper_script.append((float(t_str), action.strip()))
        traj_record(args.name, args.desc, rate_hz=args.rate,
                    record_gripper=not args.no_gripper,
                    duration=args.duration,
                    gripper_script=gripper_script)

    elif args.command == "traj-play":
        traj_play(args.name, speed=args.speed,
                  gripper_enabled=not args.no_gripper)

    elif args.command == "traj-list":
        traj_list()

    elif args.command == "traj-info":
        traj_info(args.name)

    elif args.command == "traj-delete":
        traj_delete(args.name)


if __name__ == "__main__":
    main()
