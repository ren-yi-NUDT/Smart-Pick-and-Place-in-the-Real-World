#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""观察位姿必须换到腕部有余量的 ZYZ 等价解, 否则抓取候选会被孪生判成不可达。

背景 (2026-10-04 实测): grasp1-4 录下来 J4 ≈ -164~-172.5°, 离真机 ±177.96° 限位只剩
~9°。腕部没有余量时, 任何需要改变夹爪朝向的抓取候选都会把 IK 逼到 ZYZ 翻转分支;
而翻转分支要 J4 从 -169° 走到 +172°, 只能穿过 ±180° 的限位死区 —— 走不过去, 孪生的
BRANCH_JUMP 保护因此正确地把候选判成不可用 (dependence/twin_inference/twin.py:530)。

实测同一个橙子同一个位姿: 原始 grasp1 (J4=-169°) 7 个候选只有 2 个能生成轨迹;
换成翻转解 (J4=+11°) 10 个里 9 个能。
"""

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from core.config import Config, wrist_headroom_pose      # noqa: E402

# 真机限位: J1-J5 ±177.96°, J6 ±359.82° (robot.py:wrap_joint_limits 的注释)
LIM = {"J1": 177.96, "J2": 177.96, "J3": 177.96,
       "J4": 177.96, "J5": 177.96, "J6": 359.82}

# 录下来的原始值 (改配置之前 robot_config.json 里的数)
RECORDED = {
    "grasp1": {"J1": 26.6, "J2": -6.9, "J3": 90.7, "J4": -169.0, "J5": -70.0, "J6": -21.2},
    "grasp2": {"J1": 7.0, "J2": -1.9, "J3": 91.2, "J4": -171.4, "J5": -57.8, "J6": -2.0},
    "grasp3": {"J1": -22.3, "J2": -7.2, "J3": 88.1, "J4": -163.8, "J5": -66.4, "J6": 64.3},
    "grasp4": {"J1": -50.0, "J2": -7.0, "J3": 83.9, "J4": -172.5, "J5": -77.8, "J6": -62.0},
}


def test_grasp1_flip_is_exact():
    """翻转就是 (J4+180, -J5, J6+180), J1-J3 不动。"""
    got = wrist_headroom_pose(RECORDED["grasp1"])
    assert got == {"J1": 26.6, "J2": -6.9, "J3": 90.7, "J4": 11.0, "J5": 70.0, "J6": 158.8}


@pytest.mark.parametrize("name", sorted(RECORDED))
def test_flip_gives_wrist_headroom(name):
    """翻转后 J4 必须离限位足够远, 这才是修的东西。"""
    got = wrist_headroom_pose(RECORDED[name])
    assert abs(got["J4"]) <= 90.0, f"{name}: 翻转后 J4={got['J4']} 还是贴着限位"
    assert LIM["J4"] - abs(got["J4"]) > 80.0, f"{name}: 腕部余量不足"


@pytest.mark.parametrize("name", sorted(RECORDED))
def test_flip_stays_inside_real_limits(name):
    """等价解也必须落在真机限位里, 否则真机执行不了。"""
    for j, v in wrist_headroom_pose(RECORDED[name]).items():
        assert abs(v) <= LIM[j] + 1e-9, f"{name}:{j}={v} 越限"


def test_flip_is_idempotent():
    """已经有余量的位姿不该被再翻一次。"""
    once = wrist_headroom_pose(RECORDED["grasp1"])
    assert wrist_headroom_pose(once) == once


def test_config_flips_observation_poses_but_not_others():
    """Config 加载时只动 grasp* 观察位姿。"""
    cfg = Config()
    for name in RECORDED:
        got = cfg.get_pose(name)
        assert abs(got["J4"]) <= 90.0, f"{name} 加载后 J4={got['J4']} 没换分支"
    # 递送位姿不该被动
    handover = cfg.get_pose("handover_pose")
    if handover is not None:
        raw = cfg.robot_config["handover_pose"]
        assert handover == raw, "递送位姿被误改"
