#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双臂交接：验证过的 4 步纯位姿复现序列（f181619，2026-07-05 实测可用）。

流程（下列基础速度乘以 ``speed`` 参数，默认 1 倍速）:
  0. 双臂并行 → 预置位 (基础 speed=15)
  1. 接收臂开爪；右臂 → 接近位 (基础 speed=10，两个方向都由右臂做接近动作)
  2. 接收臂闭合 → 1.5s → 交出臂打开
  3. 右臂 → 预置位 (基础 speed=15)
  4. 双臂并行 → home (基础 speed=30)

两个方向共用同一位姿组；``direction`` 只决定夹爪事件角色：
``left_to_right`` 右夹爪闭合接收、左夹爪打开交出，``right_to_left`` 互换。
经 ``skill.arm_for/gripper_for`` 取客户端，sim 模式自动路由 SimServer。
"""

import json
import os
import threading
import time

from termcolor import cprint


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POSES_DIR = os.path.join(PROJECT_ROOT, "recorded_poses")
SIDES = ("left", "right")


def _load_pose(side, name):
    with open(os.path.join(POSES_DIR, f"{side}.json")) as f:
        poses = json.load(f)
    joints = poses[name]["joint_angles_deg"]
    return {f"J{i + 1}": float(j) for i, j in enumerate(joints)}


def _scaled_speed(base, scale):
    """将基础速度按倍率缩放，并限制在控制器接受的 1..100。"""
    return max(1, min(100, int(round(base * scale))))


def play(name=None, speed=1, require_confirmation=True,
         direction="left_to_right", skill=None):
    """回放 4 步双臂交接位姿序列，返回 bool。

    ``name`` 为旧定时回放签名的兼容参数；``speed`` 是速度倍率，默认 1。
    ``skill`` 传入调用方 Skill 实例以复用其 arm_for/gripper_for 连接
    （sim 自动路由）；省略时真机直连 127.0.0.1 默认端口。
    """
    if direction not in ("left_to_right", "right_to_left"):
        raise ValueError(f"未知方向: {direction}")
    try:
        speed_scale = 1.0 if speed is None else float(speed)
    except (TypeError, ValueError):
        cprint(f"[dual-play] 非法速度倍率: {speed}", "red")
        return False
    if speed_scale <= 0:
        cprint(f"[dual-play] 速度倍率必须大于 0: {speed}", "red")
        return False
    preset_speed = _scaled_speed(15, speed_scale)
    approach_speed = _scaled_speed(10, speed_scale)
    home_speed = _scaled_speed(30, speed_scale)
    receiver, giver = (
        ("right", "left") if direction == "left_to_right" else ("left", "right")
    )

    try:
        left_preset = _load_pose("left", "right_to_left_handover_left_preset")
        right_preset = _load_pose("right", "right_to_left_handover_right_preset")
        right_approach = _load_pose("right", "right_to_left_handover_right_approach")
        homes = {side: _load_pose(side, "home") for side in SIDES}
    except (OSError, KeyError) as exc:
        cprint(f"[dual-play] 缺少交接位姿: {exc}", "red")
        return False

    if skill is not None:
        arms = {side: skill.arm_for(side) for side in SIDES}
        grippers = {side: skill.gripper_for(side) for side in SIDES}
    else:
        from core.arm import ArmClient
        from core.config import Config
        from core.gripper import GripperClient
        host = Config().shared.get("host", "127.0.0.1")
        arms = {
            "left": ArmClient(host, 8010, side="left"),
            "right": ArmClient(host, 8011, side="right"),
        }
        grippers = {
            "left": GripperClient(host, 8002, src="/left_gripper/movement_control"),
            "right": GripperClient(host, 8001, src="/right_gripper/movement_control"),
        }
        for side in SIDES:
            if not arms[side].connect() or not grippers[side].connect():
                cprint(f"[dual-play] {side} 臂/夹爪连接失败", "red")
                return False

    from core.config import Config
    if require_confirmation and not Config().sim_mode:
        input("[dual-play] 确认工作区安全后按 Enter 开始交接...")

    cprint(f"[dual-play] 位姿交接: {direction}（{giver}臂给{receiver}臂）", "cyan")

    # Step 0: 双臂并行 → 预置位
    presets = {"left": left_preset, "right": right_preset}
    threads = [
        threading.Thread(
            target=arms[side].move_to_named_pose,
            args=(presets[side],), kwargs={"speed": preset_speed},
        )
        for side in SIDES
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    # Step 1: 接收臂开爪 → 右臂接近位
    grippers[receiver].open()
    time.sleep(0.5)
    arms["right"].move_to_named_pose(right_approach, speed=approach_speed)

    # Step 2: 交接 — 接收臂闭合 → 1.5s → 交出臂打开
    grippers[receiver].close()
    time.sleep(1.5)
    grippers[giver].open()
    time.sleep(1.0)

    # Step 3: 右臂 → 预置位
    arms["right"].move_to_named_pose(right_preset, speed=preset_speed)

    # Step 4: 双臂并行 → home
    threads = [
        threading.Thread(
            target=arms[side].move_to_named_pose,
            args=(homes[side],), kwargs={"speed": home_speed},
        )
        for side in SIDES
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    cprint("[dual-play] 位姿交接完成（物体在接收臂夹爪中，双臂已回 home）", "green")
    return True
