#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Timed dual-arm handover trajectory recorder.

The two direct-SDK connections intentionally live in separate processes.
The SDK keeps global state, so creating two ``RoboticArm`` handles in one
Python process makes one of the handles invalid (usually the left one).

Default timed events:

  t=8s  right gripper close
  t=10s left gripper open
  t=20s stop drag-teach and command both arms to home

The parent process owns the clock, gripper commands, and output file. Each
arm worker owns exactly one SDK connection and streams joint samples back.
"""

import argparse
import json
import multiprocessing as mp
import os
import queue
import socket
import time
from datetime import datetime

from termcolor import cprint


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POSE_DIR = os.path.join(PROJECT_ROOT, "recorded_poses")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "recorded_trajectories", "dual")

ARM_CONFIG = {
    "left": {"ip": "192.168.1.19", "gripper_port": 8002},
    "right": {"ip": "192.168.1.18", "gripper_port": 8001},
}
GRIPPER_SRC = {
    "left": "/left_gripper/movement_control",
    "right": "/right_gripper/movement_control",
}
GRIPPER_CMD = {"open": [1000, 1000], "close": [0, 0]}
SIDES = ("left", "right")


def _load_home(side):
    path = os.path.join(POSE_DIR, f"{side}.json")
    with open(path, "r") as f:
        poses = json.load(f)
    joints = poses.get("home", {}).get("joint_angles_deg")
    if not isinstance(joints, list) or len(joints) != 7:
        raise RuntimeError(f"{path} 中缺少有效 home 位姿")
    return [float(j) for j in joints]


def _send_gripper(side, action):
    request = {
        "src": GRIPPER_SRC[side],
        "type": "set",
        "cmd": list(GRIPPER_CMD[action]),
    }
    try:
        with socket.create_connection(
            ("127.0.0.1", ARM_CONFIG[side]["gripper_port"]), 1.0
        ) as sock:
            sock.sendall(json.dumps(request).encode("utf-8"))
            sock.recv(1024)
        cprint(f"[dual-record] {side} gripper -> {action}", "yellow")
        return True
    except Exception as exc:
        cprint(f"[dual-record] {side} gripper {action} 失败: {exc}", "red")
        return False


def _at_home(joints, home, tolerance_deg=1.5):
    return joints is not None and len(joints) == 7 and all(
        abs(float(a) - float(b)) <= tolerance_deg
        for a, b in zip(joints, home)
    )


def _arm_worker(side, command_queue, result_queue, rate_hz, home):
    """Own one SDK handle and communicate with the parent via queues."""
    # Import inside the child so SDK globals are not initialized in the parent
    # and, with spawn, are never shared between the two arm connections.
    from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

    arm = None
    drag_started = False
    home_started = False
    home_reported = False
    try:
        arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
        handle = arm.rm_create_robot_arm(ARM_CONFIG[side]["ip"], 8080, level=3)
        if handle.id == -1:
            raise RuntimeError(f"无法连接{side}臂 {ARM_CONFIG[side]['ip']}:8080")
        result_queue.put({"kind": "ready", "side": side, "handle_id": handle.id})

        command = command_queue.get(timeout=120)
        if command.get("cmd") != "start":
            raise RuntimeError("录制启动前收到无效命令")

        start_tag = arm.rm_start_drag_teach(trajectory_record=1)
        if start_tag != 0:
            raise RuntimeError(f"启动拖拽示教失败，返回码 {start_tag}")
        drag_started = True
        t0 = float(command["t0"])
        result_queue.put({"kind": "started", "side": side})

        interval = 1.0 / rate_hz
        next_sample = time.monotonic()
        sequence = 0
        running = True
        while running:
            # Check commands before each sample so the home transition happens
            # promptly at the requested time.
            while True:
                try:
                    command = command_queue.get_nowait()
                except queue.Empty:
                    break

                cmd = command.get("cmd")
                if cmd == "home" and not home_started:
                    stop_tag = arm.rm_stop_drag_teach()
                    drag_started = False
                    move_tag = arm.rm_movej(
                        joint=home, v=20, r=0, connect=0, block=0
                    )
                    home_started = True
                    result_queue.put({
                        "kind": "home_sent",
                        "side": side,
                        "stop_tag": stop_tag,
                        "move_tag": move_tag,
                    })
                    if move_tag != 0:
                        raise RuntimeError(
                            f"发送 home 失败，stop_tag={stop_tag}, move_tag={move_tag}"
                        )
                elif cmd == "stop":
                    running = False
                    break

            if not running:
                break

            now = time.monotonic()
            if now < next_sample:
                time.sleep(min(next_sample - now, 0.002))
                continue

            tag, joints = arm.rm_get_joint_degree()
            if tag != 0 or not isinstance(joints, (list, tuple)) or len(joints) != 7:
                result_queue.put({
                    "kind": "error",
                    "side": side,
                    "message": f"读取关节状态失败，返回码 {tag}",
                })
                break

            result_queue.put({
                "kind": "sample",
                "side": side,
                "sequence": sequence,
                "elapsed_ms": round((now - t0) * 1000.0, 1),
                "joints": [round(float(j), 3) for j in joints],
            })
            if home_started and not home_reported and _at_home(joints, home):
                home_reported = True
                result_queue.put({
                    "kind": "home_reached",
                    "side": side,
                    "joints": [round(float(j), 3) for j in joints],
                })
            sequence += 1
            next_sample += interval
            if next_sample < now - interval:
                next_sample = now + interval

    except Exception as exc:
        result_queue.put({"kind": "error", "side": side, "message": str(exc)})
    finally:
        if arm is not None:
            if drag_started:
                try:
                    arm.rm_stop_drag_teach()
                except Exception:
                    pass
            try:
                arm.rm_delete_robot_arm()
            except Exception:
                pass
        result_queue.put({"kind": "done", "side": side})


def _start_workers(ctx, rate_hz, home):
    result_queue = ctx.Queue()
    command_queues = {side: ctx.Queue() for side in SIDES}
    workers = {
        side: ctx.Process(
            target=_arm_worker,
            args=(side, command_queues[side], result_queue, rate_hz, home[side]),
            name=f"dual-record-{side}",
        )
        for side in SIDES
    }
    try:
        for worker in workers.values():
            worker.start()

        ready = set()
        deadline = time.monotonic() + 15.0
        pending = []
        while ready != set(SIDES) and time.monotonic() < deadline:
            try:
                message = result_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            pending.append(message)
            if message["kind"] == "ready":
                ready.add(message["side"])
            elif message["kind"] == "error":
                raise RuntimeError(
                    f"{message['side']} 臂初始化失败: {message['message']}"
                )

        if ready != set(SIDES):
            raise RuntimeError(f"等待双臂 SDK 连接超时，已连接: {sorted(ready)}")
        for message in pending:
            if message["kind"] == "ready":
                cprint(
                    f"[dual-record] 已连接 {message['side']} 臂, "
                    f"ID={message['handle_id']}",
                    "green",
                )
        return workers, command_queues, result_queue
    except Exception:
        _stop_workers(workers, command_queues, result_queue)
        raise


def _stop_workers(workers, command_queues, result_queue):
    for side in SIDES:
        try:
            command_queues[side].put({"cmd": "stop"})
        except Exception:
            pass
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline and any(
        worker.is_alive() for worker in workers.values()
    ):
        try:
            result_queue.get(timeout=0.2)
        except queue.Empty:
            pass
    for worker in workers.values():
        worker.join(timeout=0.5)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=1.0)


def record(name, description="", rate_hz=50, right_close_at=8.0,
           left_open_at=10.0, home_at=20.0):
    rate_hz = max(10, min(100, int(rate_hz)))
    if min(right_close_at, left_open_at, home_at) < 0:
        raise ValueError("事件时间不能为负数")
    if home_at <= max(right_close_at, left_open_at):
        raise ValueError("home 时间必须晚于夹爪事件时间")

    home = {side: _load_home(side) for side in SIDES}
    ctx = mp.get_context("spawn")
    workers = command_queues = result_queue = None
    samples = {side: {} for side in SIDES}
    latest = {side: None for side in SIDES}
    events = []
    home_started = False
    home_reached_since = None
    home_deadline = None
    home_reached = set()
    started = set()
    interrupted = False
    failure = None

    try:
        workers, command_queues, result_queue = _start_workers(ctx, rate_hz, home)
        input("[dual-record] 确认两臂及工作区安全，按 Enter 开始拖拽录制...")

        t0 = time.monotonic()
        for side in SIDES:
            command_queues[side].put({"cmd": "start", "t0": t0})

        start_deadline = time.monotonic() + 10.0
        while started != set(SIDES) and time.monotonic() < start_deadline:
            try:
                message = result_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if message["kind"] == "started":
                started.add(message["side"])
            elif message["kind"] == "error":
                raise RuntimeError(
                    f"启动拖拽示教失败: {message['side']}={message['message']}"
                )
        if started != set(SIDES):
            raise RuntimeError(f"启动拖拽示教超时，已启动: {sorted(started)}")

        interval = 1.0 / rate_hz
        next_tick = time.monotonic()
        cprint(
            f"[dual-record] 开始：t={right_close_at:g}s 右夹爪闭合，"
            f"t={left_open_at:g}s 左夹爪打开，t={home_at:g}s 双臂回 home",
            "green",
        )

        while True:
            now = time.monotonic()
            elapsed = now - t0

            if not any(e["event"] == "right_close" for e in events) and elapsed >= right_close_at:
                if not _send_gripper("right", "close"):
                    raise RuntimeError("右夹爪闭合失败")
                events.append({"time_s": round(elapsed, 3), "event": "right_close"})

            if not any(e["event"] == "left_open" for e in events) and elapsed >= left_open_at:
                if not _send_gripper("left", "open"):
                    raise RuntimeError("左夹爪打开失败")
                events.append({"time_s": round(elapsed, 3), "event": "left_open"})

            if not home_started and elapsed >= home_at:
                cprint("[dual-record] 到达 home 触发时间，停止拖拽并发送双臂 home", "cyan")
                for side in SIDES:
                    command_queues[side].put({"cmd": "home"})
                events.append({"time_s": round(elapsed, 3), "event": "both_home"})
                home_started = True
                home_deadline = time.monotonic() + 60.0

            while True:
                try:
                    message = result_queue.get_nowait()
                except queue.Empty:
                    break
                kind = message["kind"]
                side = message.get("side")
                if kind == "sample":
                    sequence = message["sequence"]
                    samples[side][sequence] = message
                    latest[side] = message["joints"]
                elif kind == "home_sent":
                    cprint(
                        f"[dual-record] {side} home 已发送 "
                        f"(stop_tag={message['stop_tag']}, "
                        f"move_tag={message['move_tag']})",
                        "cyan",
                    )
                elif kind == "home_reached":
                    home_reached.add(side)
                    cprint(f"[dual-record] {side} 已确认到达 home", "green")
                elif kind == "error":
                    raise RuntimeError(f"{side} 臂录制失败: {message['message']}")

            if home_started and home_reached == set(SIDES):
                if home_reached_since is None:
                    home_reached_since = time.monotonic()
                elif time.monotonic() - home_reached_since >= 0.5:
                    break
            else:
                home_reached_since = None

            if home_deadline is not None and time.monotonic() > home_deadline:
                status = {
                    side: {
                        "latest": latest[side],
                        "home": home[side],
                        "max_error_deg": (
                            None
                            if latest[side] is None
                            else max(
                                abs(float(a) - float(b))
                                for a, b in zip(latest[side], home[side])
                            )
                        ),
                        "home_confirmed": side in home_reached,
                    }
                    for side in SIDES
                }
                raise RuntimeError(f"双臂回 home 超时（60 秒）: {status}")

            next_tick += interval
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                time.sleep(0.002)

    except KeyboardInterrupt:
        interrupted = True
        cprint("[dual-record] 用户中断，保存已采样部分", "yellow")
    except Exception as exc:
        failure = str(exc)
        cprint(f"[dual-record] 录制失败: {failure}", "red")
    finally:
        if command_queues is not None:
            _stop_workers(workers, command_queues, result_queue)

    common_sequences = sorted(set(samples["left"]) & set(samples["right"]))
    waypoints = [
        {
            "elapsed_ms": samples["left"][sequence]["elapsed_ms"],
            "left": samples["left"][sequence]["joints"],
            "right": samples["right"][sequence]["joints"],
        }
        for sequence in common_sequences
    ]
    if not waypoints:
        cprint("[dual-record] 没有双臂同步采样点，未保存", "red")
        return False

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    output = {
        "name": name,
        "description": description,
        "type": "timed_dual_arm_handover",
        "timestamp": datetime.now().isoformat(),
        "recording_rate_hz": rate_hz,
        "num_points": len(waypoints),
        "duration_ms": waypoints[-1]["elapsed_ms"],
        "events": events,
        "home_reference": home,
        "waypoints": waypoints,
    }
    if failure:
        output["failure"] = failure
        path = os.path.join(
            OUTPUT_DIR,
            f"{name}.failed_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
        )
    else:
        path = os.path.join(OUTPUT_DIR, f"{name}.json")
    with open(path, "w") as f:
        json.dump(output, f, indent=2, ensure_ascii=False)
    cprint(f"[dual-record] 已保存: {path}", "green")
    cprint(
        f"[dual-record] 时长: {output['duration_ms'] / 1000.0:.1f}s, "
        f"点数: {len(waypoints)}",
        "cyan",
    )
    cprint(f"[dual-record] 事件: {events}", "cyan")
    if failure:
        cprint(f"[dual-record] 失败现场已保存: {path}", "yellow")
        return False
    if interrupted:
        cprint("[dual-record] 这是用户中断时保存的部分轨迹", "yellow")
    return True


def main():
    parser = argparse.ArgumentParser(description="双臂定时交接连续轨迹录制")
    sub = parser.add_subparsers(dest="command", required=True)
    rec = sub.add_parser("record", help="开始双臂拖拽录制")
    rec.add_argument("--name", required=True)
    rec.add_argument("--desc", default="双臂单段轨迹：8秒右闭、10秒左开、20秒双臂home")
    rec.add_argument("--rate", type=int, default=50)
    rec.add_argument("--right-close-at", type=float, default=8.0)
    rec.add_argument("--left-open-at", type=float, default=10.0)
    rec.add_argument("--home-at", type=float, default=20.0)
    args = parser.parse_args()
    if args.command == "record":
        ok = record(
            args.name,
            args.desc,
            args.rate,
            args.right_close_at,
            args.left_open_at,
            args.home_at,
        )
        raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    mp.freeze_support()
    main()
