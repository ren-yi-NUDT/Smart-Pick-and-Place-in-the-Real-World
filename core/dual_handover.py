#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Replay a timed dual-arm trajectory with one SDK handle per process."""

import argparse
import json
import math
import multiprocessing as mp
import os
import queue
import socket
import time

from termcolor import cprint


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRAJ_DIR = os.path.join(PROJECT_ROOT, "recorded_trajectories", "dual")
ARM_CONFIG = {
    "left": {"ip": "192.168.1.19", "gripper_port": 8002},
    "right": {"ip": "192.168.1.18", "gripper_port": 8001},
}
GRIPPER_SRC = {"left": "/left_gripper/movement_control", "right": "/right_gripper/movement_control"}
GRIPPER_CMD = {"open": [1000, 1000], "close": [0, 0]}
SIDES = ("left", "right")
DEFAULT_SPEEDS = {
    "dual_handover_timed_20260826_v2": 0.9,
}


def _send_gripper(side, action):
    request = {"src": GRIPPER_SRC[side], "type": "set", "cmd": GRIPPER_CMD[action]}
    try:
        with socket.create_connection(("127.0.0.1", ARM_CONFIG[side]["gripper_port"]), 1.0) as sock:
            sock.sendall(json.dumps(request).encode("utf-8"))
            sock.recv(1024)
        cprint(f"[dual-play] {side} gripper -> {action}", "yellow")
        return True
    except Exception as exc:
        cprint(f"[dual-play] {side} gripper {action} 失败: {exc}", "red")
        return False


def _healthy(arm):
    tag, state = arm.rm_get_current_arm_state()
    if tag != 0 or not isinstance(state, dict):
        return False
    return not state.get("arm_err", 0) and not state.get("sys_err", 0)


def _slow_stop(arm):
    try:
        arm.rm_set_arm_slow_stop()
    except Exception:
        pass


def _at_target(arm, target, tolerance=1.5):
    tag, joints = arm.rm_get_joint_degree()
    return tag == 0 and len(joints) == 7 and all(
        abs(float(a) - float(b)) <= tolerance for a, b in zip(joints, target)
    )


def _worker(side, command_queue, result_queue, start_joints, trajectory, speed):
    from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

    arm = None
    motion_started = False
    try:
        arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
        handle = arm.rm_create_robot_arm(ARM_CONFIG[side]["ip"], 8080, level=3)
        if handle.id == -1:
            raise RuntimeError(f"无法连接{side}臂 {ARM_CONFIG[side]['ip']}:8080")
        result_queue.put({"kind": "ready", "side": side, "id": handle.id})

        command = command_queue.get(timeout=30)
        if command.get("cmd") != "prepare":
            raise RuntimeError("收到无效的准备命令")
        if not _healthy(arm):
            raise RuntimeError("准备回放前机械臂状态异常")
        if not _at_target(arm, start_joints):
            tag = arm.rm_movej(joint=start_joints, v=15, r=0, connect=0, block=0)
            if tag != 0:
                raise RuntimeError(f"运动到轨迹起点失败，返回码 {tag}")
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline and not _at_target(arm, start_joints):
                if not _healthy(arm):
                    raise RuntimeError("运动到轨迹起点时机械臂状态异常")
                time.sleep(0.05)
            if not _at_target(arm, start_joints):
                raise RuntimeError("运动到轨迹起点超时")
        result_queue.put({"kind": "prepared", "side": side})

        command = command_queue.get(timeout=30)
        if command.get("cmd") != "play":
            raise RuntimeError("收到无效的回放命令")
        t0 = float(command["t0"])
        effective_interval = max(
            0.005, (trajectory[1][0] - trajectory[0][0]) / 1000.0 / speed
        )
        check_every = max(1, round(0.2 / effective_interval))
        use_follow = effective_interval <= 0.01
        for index, (offset_ms, joints) in enumerate(trajectory):
            target_time = t0 + offset_ms / 1000.0 / speed
            while True:
                try:
                    stop_cmd = command_queue.get_nowait()
                except queue.Empty:
                    stop_cmd = None
                if stop_cmd and stop_cmd.get("cmd") == "stop":
                    _slow_stop(arm)
                    result_queue.put({"kind": "stopped", "side": side})
                    return
                remaining = target_time - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(remaining, 0.005))

            tag = arm.rm_movej_canfd(joint=joints, follow=use_follow, expand=0)
            motion_started = True
            if tag != 0:
                raise RuntimeError(f"CAN-FD 发送失败，点 {index + 1}/{len(trajectory)}，返回码 {tag}")
            if index % check_every == 0 and not _healthy(arm):
                raise RuntimeError(f"回放过程中机械臂状态异常，点 {index + 1}")
            if (index + 1) % max(1, len(trajectory) // 10) == 0:
                cprint(f"[dual-play] {side}: {(index + 1) * 100 // len(trajectory)}%", "yellow")

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if _at_target(arm, trajectory[-1][1]):
                result_queue.put({"kind": "finished", "side": side})
                return
            time.sleep(0.05)
        raise RuntimeError("轨迹终点未确认")
    except Exception as exc:
        if motion_started and arm is not None:
            _slow_stop(arm)
        result_queue.put({"kind": "error", "side": side, "message": str(exc)})
    finally:
        if arm is not None:
            try:
                arm.rm_delete_robot_arm()
            except Exception:
                pass
        result_queue.put({"kind": "done", "side": side})


def _load(path):
    with open(path, "r") as f:
        data = json.load(f)
    if data.get("type") != "timed_dual_arm_handover":
        raise ValueError("不是 timed_dual_arm_handover 双臂轨迹")
    raw = data.get("waypoints")
    if not isinstance(raw, list) or len(raw) < 2:
        raise ValueError("双臂轨迹点不足")
    first = float(raw[0]["elapsed_ms"])
    trajectory = []
    previous = -1.0
    for index, point in enumerate(raw):
        elapsed = float(point["elapsed_ms"]) - first
        left = [float(x) for x in point["left"]]
        right = [float(x) for x in point["right"]]
        if len(left) != 7 or len(right) != 7 or elapsed <= previous:
            raise ValueError(f"第 {index + 1} 个双臂轨迹点无效")
        if not all(math.isfinite(x) for x in left + right + [elapsed]):
            raise ValueError(f"第 {index + 1} 个双臂轨迹点包含非有限数值")
        trajectory.append((elapsed, left, right))
        previous = elapsed
    return data, trajectory


def _resample(trajectory, step_ms=10.0):
    """Linearly resample a trajectory for stable high-follow CAN-FD output."""
    if len(trajectory) < 2:
        return trajectory
    end_ms = trajectory[-1][0]
    times = []
    t = 0.0
    while t < end_ms:
        times.append(t)
        t += step_ms
    if not times or times[-1] != end_ms:
        times.append(end_ms)

    result = []
    segment = 0
    for target_ms in times:
        while segment + 1 < len(trajectory) and trajectory[segment + 1][0] < target_ms:
            segment += 1
        if segment + 1 >= len(trajectory):
            result.append((target_ms, list(trajectory[-1][1])))
            continue
        t0, q0 = trajectory[segment]
        t1, q1 = trajectory[segment + 1]
        if t1 == t0:
            ratio = 0.0
        else:
            ratio = (target_ms - t0) / (t1 - t0)
        joints = [a + (b - a) * ratio for a, b in zip(q0, q1)]
        result.append((target_ms, joints))
    return result


def play(name, speed=None, require_confirmation=True, direction="left_to_right"):
    """回放双臂定时交接轨迹。

    臂部航点两个方向共用同一录制；方向只决定夹爪事件的角色：
    ``left_to_right`` 按文件原义回放（右闭接、左开放），``right_to_left``
    将文件中的 close/open 事件互换到对侧臂，时刻不变。
    """
    if direction not in ("left_to_right", "right_to_left"):
        raise ValueError(f"未知方向: {direction}")
    receiver, giver = (
        ("right", "left") if direction == "left_to_right" else ("left", "right")
    )
    close_event, open_event = f"{receiver}_close", f"{giver}_open"
    path = os.path.join(TRAJ_DIR, f"{name}.json")
    if not os.path.isfile(path):
        raise FileNotFoundError(path)
    if speed is None:
        speed = DEFAULT_SPEEDS.get(name, 0.5)
    if speed <= 0 or not math.isfinite(speed):
        raise ValueError("速度倍率必须大于 0")
    data, raw = _load(path)
    trajectories = {
        "left": [(t, left) for t, left, _ in raw],
        "right": [(t, right) for t, _, right in raw],
    }
    original_interval = raw[1][0] - raw[0][0]
    effective_interval = original_interval / speed
    if effective_interval > 10.0:
        trajectories = {
            side: _resample(trajectory, step_ms=10.0)
            for side, trajectory in trajectories.items()
        }
        cprint(
            f"[dual-play] 已将轨迹插值到 10ms 周期，减少透传目标跳变 "
            f"(原始间隔 {original_interval:g}ms)",
            "cyan",
        )
    starts = {side: trajectories[side][0][1] for side in SIDES}
    cprint(f"[dual-play] 轨迹: {path}", "cyan")
    cprint(f"[dual-play] 原始时长: {raw[-1][0] / 1000.0:.1f}s, 速度: {speed:g}x", "cyan")
    cprint(f"[dual-play] 文件事件: {data.get('events', [])}", "cyan")
    cprint(
        f"[dual-play] 方向: {direction}（{giver}臂给{receiver}臂）", "cyan"
    )

    ctx = mp.get_context("spawn")
    result_queue = ctx.Queue()
    command_queues = {side: ctx.Queue() for side in SIDES}
    workers = {
        side: ctx.Process(
            target=_worker,
            args=(side, command_queues[side], result_queue, starts[side], trajectories[side], speed),
            name=f"dual-play-{side}",
        )
        for side in SIDES
    }
    try:
        for worker in workers.values():
            worker.start()
        ready = set()
        while ready != set(SIDES):
            message = result_queue.get(timeout=15.0)
            if message["kind"] == "ready":
                ready.add(message["side"])
                cprint(f"[dual-play] 已连接 {message['side']} 臂, ID={message['id']}", "green")
            elif message["kind"] == "error":
                raise RuntimeError(f"{message['side']} 臂连接失败: {message['message']}")

        for side in SIDES:
            command_queues[side].put({"cmd": "prepare"})
        prepared = set()
        while prepared != set(SIDES):
            message = result_queue.get(timeout=35.0)
            if message["kind"] == "prepared":
                prepared.add(message["side"])
                cprint(f"[dual-play] {message['side']} 臂已到达起点", "green")
            elif message["kind"] == "error":
                raise RuntimeError(f"{message['side']} 臂准备失败: {message['message']}")

        if require_confirmation:
            input("[dual-play] 确认工作区安全、夹爪无障碍，按 Enter 开始回放...")
        t0 = time.monotonic()
        for side in SIDES:
            command_queues[side].put({"cmd": "play", "t0": t0})

        event_done = set()
        file_times = {
            event["event"]: float(event["time_s"]) / speed
            for event in data.get("events", [])
        }
        # 文件事件按录制方向命名（right_close/left_open）；
        # 反向回放时沿用同一时刻，把角色换到对侧臂。
        event_times = {
            close_event: file_times.get("right_close", float("inf")),
            open_event: file_times.get("left_open", float("inf")),
        }
        finished = set()
        while finished != set(SIDES):
            elapsed = time.monotonic() - t0
            if close_event not in event_done and elapsed >= event_times[close_event]:
                if not _send_gripper(receiver, "close"):
                    raise RuntimeError(f"{receiver} 夹爪闭合失败")
                event_done.add(close_event)
            if open_event not in event_done and elapsed >= event_times[open_event]:
                if not _send_gripper(giver, "open"):
                    raise RuntimeError(f"{giver} 夹爪打开失败")
                event_done.add(open_event)

            try:
                message = result_queue.get(timeout=0.2)
            except queue.Empty:
                # Workers may be between CAN-FD packets; keep the parent
                # clock running so timed gripper events are not skipped.
                continue
            if message["kind"] == "finished":
                finished.add(message["side"])
            elif message["kind"] == "error":
                raise RuntimeError(f"{message['side']} 臂回放失败: {message['message']}")
        cprint("[dual-play] 双臂回放完成，终点已确认", "green")
        return True
    except KeyboardInterrupt:
        cprint("[dual-play] 用户中断，执行缓停", "yellow")
        return False
    finally:
        for side in SIDES:
            try:
                command_queues[side].put({"cmd": "stop"})
            except Exception:
                pass
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and any(worker.is_alive() for worker in workers.values()):
            try:
                result_queue.get(timeout=0.2)
            except queue.Empty:
                pass
        for worker in workers.values():
            worker.join(timeout=0.5)
            if worker.is_alive():
                worker.terminate()
                worker.join(timeout=1.0)


def main():
    parser = argparse.ArgumentParser(description="双臂定时交接轨迹回放")
    parser.add_argument("--name", required=True)
    parser.add_argument(
        "--speed", type=float, default=None,
        help="速度倍率；当前 v2 交接轨迹默认 0.9，其它轨迹默认 0.5",
    )
    parser.add_argument(
        "--direction", choices=("left_to_right", "right_to_left"),
        default="left_to_right",
        help="交接方向（决定夹爪事件角色，航点两个方向共用）",
    )
    args = parser.parse_args()
    raise SystemExit(
        0 if play(args.name, args.speed, direction=args.direction) else 1
    )


if __name__ == "__main__":
    mp.freeze_support()
    main()
