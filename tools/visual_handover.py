#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四段轨迹 + 视觉确认的双臂交接状态机。

流程:
  1. 左臂回放交接轨迹1，右臂回放交接轨迹2；
  2. 右臂在轨迹2终点原地拍摄并视觉抓取左臂上的物品；
  3. 右夹爪通过 object_detected 确认抓住后保持，左夹爪才打开；
  4. 左臂回放交接轨迹3；右臂默认保持抓取位并跳过交接轨迹4；
  5. 从两臂当前关节状态生成五次多项式平滑轨迹，双臂回 home。

默认轨迹文件:
  recorded_trajectories/left/handover_1.json
  recorded_trajectories/right/handover_2.json
  recorded_trajectories/left/handover_3.json
  recorded_trajectories/right/handover_4.json (可选，使用 --use-right-traj4 启用)

建议从项目根目录、以可用硬件环境执行:
  python -m tools.visual_handover --object 皮卡丘玩偶
"""

import argparse
import json
import math
import multiprocessing as mp
import os
import queue
import socket
import time

from termcolor import cprint

from skills.grasp import GraspSkill
from tools.pose_record import (
    FINAL_POSITION_TOLERANCE_DEG,
    START_POSITION_TOLERANCE_DEG,
    START_SETTLE_TIME_S,
    _check_arm_health,
    _connect_arm,
    _load_home_joints,
    _load_trajectory,
    _move_to_start,
    _playback_canfd,
    _safe_slow_stop,
    traj_play,
    _validate_waypoints,
)


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARM_IPS = {"left": "192.168.1.19", "right": "192.168.1.18"}
GRIPPER_PORTS = {"left": 8002, "right": 8001}
GRIPPER_SRCS = {
    "left": "/left_gripper/movement_control",
    "right": "/right_gripper/movement_control",
}
GRIPPER_COMMANDS = {"open": [1000, 1000], "close": [0, 0]}
DEFAULT_TRAJECTORIES = {
    "left1": "handover_1",
    "right2": "handover_2",
    "left3": "handover_3",
    "right4": "handover_4",
}
DEFAULT_SPEED = 0.9
PAIR_START_TIMEOUT_S = 45.0
PAIR_PLAY_TIMEOUT_MARGIN_S = 20.0
SMOOTH_HOME_STEP_MS = 10.0


def _trajectory(name, side):
    """Load and validate one side's JSON trajectory."""
    data = _load_trajectory(name, side)
    expected = os.path.join(
        PROJECT_ROOT, "recorded_trajectories", side, f"{name}.json"
    )
    if not isinstance(data, dict):
        raise FileNotFoundError(f"未找到{side}臂轨迹: {expected}")
    waypoints = _validate_waypoints(data.get("waypoints"))
    if data.get("arm") not in (None, side):
        raise ValueError(f"轨迹 {expected} 的 arm 字段不是 {side}")
    return data, waypoints


def _at_target(arm, target, tolerance=START_POSITION_TOLERANCE_DEG):
    try:
        tag, joints = arm.rm_get_joint_degree()
        return (
            tag == 0
            and len(joints) == 7
            and all(abs(float(a) - float(b)) <= tolerance for a, b in zip(joints, target))
        )
    except Exception:
        return False


def _pair_worker(side, command_queue, result_queue, start_joints, trajectory, speed):
    """One SDK process for one arm; both workers share the same playback clock."""
    from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

    arm = None
    motion_started = False
    try:
        arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
        handle = arm.rm_create_robot_arm(ARM_IPS[side], 8080, level=3)
        if handle.id == -1:
            raise RuntimeError(f"无法连接{side}臂 {ARM_IPS[side]}:8080")
        result_queue.put({"kind": "ready", "side": side, "id": handle.id})

        command = command_queue.get(timeout=PAIR_START_TIMEOUT_S)
        if command.get("cmd") != "prepare":
            raise RuntimeError("收到无效的准备命令")
        healthy, reason = _check_arm_health(arm)
        if not healthy:
            raise RuntimeError(f"准备前机械臂状态异常: {reason}")
        if not _at_target(arm, start_joints):
            if not _move_to_start(arm, start_joints):
                raise RuntimeError("运动到轨迹起点失败")
        if START_SETTLE_TIME_S > 0:
            time.sleep(START_SETTLE_TIME_S)
        result_queue.put({"kind": "prepared", "side": side})

        command = command_queue.get(timeout=PAIR_START_TIMEOUT_S)
        if command.get("cmd") != "play":
            raise RuntimeError("收到无效的回放命令")
        t0 = float(command["t0"])
        speed = float(speed)
        effective_interval = max(
            0.005,
            (trajectory[1][0] - trajectory[0][0]) / 1000.0 / speed,
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
                    _safe_slow_stop(arm)
                    result_queue.put({"kind": "stopped", "side": side})
                    return
                remaining = target_time - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(remaining, 0.005))

            tag = arm.rm_movej_canfd(joint=joints, follow=use_follow, expand=0)
            motion_started = True
            if tag != 0:
                raise RuntimeError(
                    f"CAN-FD 发送失败，点 {index + 1}/{len(trajectory)}，返回码 {tag}"
                )
            if index % check_every == 0:
                healthy, reason = _check_arm_health(arm)
                if not healthy:
                    raise RuntimeError(f"回放过程中机械臂状态异常: {reason}")

        deadline = time.monotonic() + max(
            8.0, PAIR_PLAY_TIMEOUT_MARGIN_S + trajectory[-1][0] / 1000.0 / speed
        )
        while time.monotonic() < deadline:
            if _at_target(arm, trajectory[-1][1], FINAL_POSITION_TOLERANCE_DEG):
                result_queue.put({"kind": "finished", "side": side})
                return
            time.sleep(0.05)
        raise RuntimeError("轨迹终点未确认")
    except Exception as exc:
        if motion_started and arm is not None:
            _safe_slow_stop(arm)
        result_queue.put({"kind": "error", "side": side, "message": str(exc)})
    finally:
        if arm is not None:
            try:
                arm.rm_delete_robot_arm()
            except Exception:
                pass
        result_queue.put({"kind": "done", "side": side})


def _stop_workers(workers, command_queues):
    for command_queue in command_queues.values():
        try:
            command_queue.put({"cmd": "stop"})
        except Exception:
            pass
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline and any(worker.is_alive() for worker in workers.values()):
        time.sleep(0.1)
    for worker in workers.values():
        worker.join(timeout=0.5)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=1.0)


def _play_pair_waypoints(left_waypoints, right_waypoints, speed, label):
    """Play two validated trajectories concurrently with per-arm health checks."""
    if speed <= 0 or not math.isfinite(float(speed)):
        raise ValueError("速度倍率必须为正数")
    cprint(f"[handover] 并行回放: {label}, 速度 {speed:g}x", "cyan")

    ctx = mp.get_context("spawn")
    result_queue = ctx.Queue()
    command_queues = {side: ctx.Queue() for side in ("left", "right")}
    trajectories = {"left": left_waypoints, "right": right_waypoints}
    starts = {side: trajectories[side][0][1] for side in trajectories}
    workers = {
        side: ctx.Process(
            target=_pair_worker,
            args=(
                side,
                command_queues[side],
                result_queue,
                starts[side],
                trajectories[side],
                speed,
            ),
            name=f"handover-{label}-{side}",
        )
        for side in trajectories
    }

    try:
        for worker in workers.values():
            worker.start()
        ready = set()
        deadline = time.monotonic() + PAIR_START_TIMEOUT_S
        while ready != set(trajectories) and time.monotonic() < deadline:
            try:
                message = result_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if message["kind"] == "ready":
                ready.add(message["side"])
                cprint(f"[handover] {message['side']} 臂已连接", "green")
            elif message["kind"] == "error":
                raise RuntimeError(f"{message['side']} 臂准备失败: {message['message']}")
        if ready != set(trajectories):
            raise RuntimeError("等待双臂连接超时")

        for command_queue in command_queues.values():
            command_queue.put({"cmd": "prepare"})
        prepared = set()
        deadline = time.monotonic() + PAIR_START_TIMEOUT_S
        while prepared != set(trajectories) and time.monotonic() < deadline:
            try:
                message = result_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if message["kind"] == "prepared":
                prepared.add(message["side"])
                cprint(f"[handover] {message['side']} 臂已到达轨迹起点", "green")
            elif message["kind"] == "error":
                raise RuntimeError(f"{message['side']} 臂准备失败: {message['message']}")
        if prepared != set(trajectories):
            raise RuntimeError("双臂到达轨迹起点超时")

        t0 = time.monotonic() + 0.5
        for command_queue in command_queues.values():
            command_queue.put({"cmd": "play", "t0": t0})

        finished = set()
        expected = set(trajectories)
        while finished != expected:
            try:
                message = result_queue.get(timeout=1.0)
            except queue.Empty:
                if not any(worker.is_alive() for worker in workers.values()):
                    raise RuntimeError("双臂回放进程意外退出")
                continue
            if message["kind"] == "finished":
                finished.add(message["side"])
            elif message["kind"] == "error":
                raise RuntimeError(f"{message['side']} 臂回放失败: {message['message']}")
        cprint(f"[handover] {label} 双臂轨迹完成", "green")
        return True
    except KeyboardInterrupt:
        cprint("[handover] 用户中断，执行缓停", "yellow")
        return False
    except Exception as exc:
        cprint(f"[handover] {label} 失败: {exc}", "red")
        return False
    finally:
        _stop_workers(workers, command_queues)


def _gripper_request(side, action=None):
    request = {
        "src": GRIPPER_SRCS[side],
        "type": "get" if action is None else "set",
    }
    if action is not None:
        request["cmd"] = GRIPPER_COMMANDS[action]
    try:
        with socket.create_connection(("127.0.0.1", GRIPPER_PORTS[side]), timeout=3.0) as sock:
            sock.sendall(json.dumps(request).encode("utf-8"))
            response = json.loads(sock.recv(4096).decode("utf-8"))
        if response.get("value") is False:
            raise RuntimeError(response)
        return response
    except Exception as exc:
        raise RuntimeError(f"{side} 夹爪服务不可用: {exc}") from exc


def _check_preflight():
    for side in ("left", "right"):
        arm, handle = _connect_arm(side)
        if arm is None or handle is None:
            return False
        try:
            healthy, reason = _check_arm_health(arm)
            if not healthy:
                cprint(f"[handover] {side} 臂不健康: {reason}", "red")
                return False
        finally:
            try:
                arm.rm_delete_robot_arm()
            except Exception:
                pass
        try:
            response = _gripper_request(side)
            cprint(f"[handover] {side} 夹爪: {response.get('info', response)}", "green")
        except RuntimeError as exc:
            cprint(f"[handover] {exc}", "red")
            return False
    return True


def _smooth_home_waypoints(current, target):
    """Generate a zero-velocity/zero-acceleration quintic joint trajectory."""
    max_delta = max(abs(float(a) - float(b)) for a, b in zip(current, target))
    duration_s = max(3.0, min(15.0, max_delta / 15.0 if max_delta else 3.0))
    count = max(2, int(math.ceil(duration_s * 1000.0 / SMOOTH_HOME_STEP_MS)))
    duration_ms = count * SMOOTH_HOME_STEP_MS
    waypoints = []
    for index in range(count + 1):
        u = index / count
        blend = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        joints = [
            float(start) + (float(end) - float(start)) * blend
            for start, end in zip(current, target)
        ]
        waypoints.append([index * SMOOTH_HOME_STEP_MS] + joints)
    cprint(
        f"[handover] 生成平滑回 home 轨迹: {duration_ms / 1000.0:.1f}s, "
        f"最大关节差 {max_delta:.1f}°",
        "cyan",
    )
    return _validate_waypoints(waypoints)


def _current_joints(side):
    arm, handle = _connect_arm(side)
    if arm is None or handle is None:
        raise RuntimeError(f"无法读取 {side} 臂当前关节")
    try:
        healthy, reason = _check_arm_health(arm)
        if not healthy:
            raise RuntimeError(f"{side} 臂状态异常: {reason}")
        tag, joints = arm.rm_get_joint_degree()
        if tag != 0 or len(joints) != 7:
            raise RuntimeError(f"{side} 臂关节读取失败(tag={tag})")
        return [float(joint) for joint in joints]
    finally:
        try:
            arm.rm_delete_robot_arm()
        except Exception:
            pass


def run(args):
    names = {
        "left1": args.left_traj1,
        "right2": args.right_traj2,
        "left3": args.left_traj3,
        "right4": args.right_traj4,
    }
    try:
        _, left1 = _trajectory(names["left1"], "left")
        _, right2 = _trajectory(names["right2"], "right")
        _, left3 = _trajectory(names["left3"], "left")
        right4 = None
        if args.use_right_traj4:
            _, right4 = _trajectory(names["right4"], "right")
    except (FileNotFoundError, TypeError, ValueError) as exc:
        cprint(f"[handover] 轨迹检查失败: {exc}", "red")
        return False

    if not _check_preflight():
        cprint("[handover] 预检查失败，未执行任何轨迹", "red")
        return False
    if not args.no_confirm:
        answer = input(
            "[handover] 确认工作区安全、物品由左臂夹持、右臂交接区无障碍，"
            "按 Enter 开始；输入 q 取消: "
        )
        if answer.strip().lower() == "q":
            cprint("[handover] 用户取消", "yellow")
            return False

    if not _play_pair_waypoints(left1, right2, args.speed, "轨迹1+2 到交接位"):
        return False

    right_observation_pose = {
        f"J{index + 1}": joints
        for index, joints in enumerate(right2[-1][1])
    }
    cprint("[handover] 右臂在轨迹2终点视觉抓取，成功后保持抓取位", "cyan")
    grasp = GraspSkill()
    if not grasp.run(
        object=args.object,
        side="right",
        location="current",
        observation_pose=right_observation_pose,
        hold_after_grasp=True,
    ):
        cprint("[handover] 右臂未确认抓住，左夹爪保持闭合，不执行释放", "red")
        return False
    try:
        state = _gripper_request("right")
        cprint(f"[handover] 右夹爪抓取确认: {state.get('info', state)}", "green")
    except RuntimeError as exc:
        cprint(f"[handover] 右夹爪状态复核失败，左夹爪保持闭合: {exc}", "red")
        return False

    try:
        left_open = _gripper_request("left", "open")
        cprint(f"[handover] 左夹爪已松开: {left_open.get('info', left_open)}", "green")
    except RuntimeError as exc:
        cprint(f"[handover] 左夹爪释放失败，右臂保持不动: {exc}", "red")
        return False

    if args.use_right_traj4:
        if not _play_pair_waypoints(left3, right4, args.speed, "轨迹3+4 离开交接区"):
            cprint("[handover] 离开交接区失败；右夹爪不自动释放物品", "red")
            return False
    else:
        cprint("[handover] 跳过右臂轨迹4，右臂保持抓取位", "cyan")
        if not traj_play(names["left3"], arm_side="left", speed=args.speed):
            cprint("[handover] 左臂轨迹3失败；右夹爪不自动释放物品", "red")
            return False

    try:
        left_current = _current_joints("left")
        right_current = _current_joints("right")
        left_home = _load_home_joints("left")
        right_home = _load_home_joints("right")
    except (RuntimeError, OSError, KeyError, ValueError) as exc:
        cprint(f"[handover] 生成平滑 home 轨迹前读取状态失败: {exc}", "red")
        return False

    left_home_path = _smooth_home_waypoints(left_current, left_home)
    right_home_path = _smooth_home_waypoints(right_current, right_home)
    if not _play_pair_waypoints(left_home_path, right_home_path, args.speed, "平滑回 home"):
        cprint("[handover] 平滑回 home 失败；右夹爪保持物品", "red")
        return False
    cprint("[handover] 全部流程完成；右夹爪保持闭合，物品未自动释放", "green")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--object", required=True, help="右臂要抓取的物品名称")
    parser.add_argument("--left-traj1", default=DEFAULT_TRAJECTORIES["left1"])
    parser.add_argument("--right-traj2", default=DEFAULT_TRAJECTORIES["right2"])
    parser.add_argument("--left-traj3", default=DEFAULT_TRAJECTORIES["left3"])
    parser.add_argument("--right-traj4", default=DEFAULT_TRAJECTORIES["right4"])
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED)
    parser.add_argument(
        "--use-right-traj4",
        action="store_true",
        help="启用右臂轨迹4；默认跳过轨迹4并保持右臂抓取位",
    )
    parser.add_argument("--no-confirm", action="store_true", help="跳过人工安全确认")
    args = parser.parse_args()
    mp.freeze_support()
    raise SystemExit(0 if run(args) else 1)


if __name__ == "__main__":
    main()
