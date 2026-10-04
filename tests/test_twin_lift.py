#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Twin 抬升路点插入的单元测试 (纯逻辑, 不连 PyBullet/硬件)。
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "dependence", "twin_inference",
))

from twin import insert_lift_waypoints  # noqa: E402

ORN = [0.0, 0.0, 0.0, 1.0]
TH = 0.10
MARGIN = 0.02


def test_low_target_reached_laterally_gets_a_lift_waypoint():
    pos, orn, lift = insert_lift_waypoints(
        [[0.5, 0.0, 0.05]], [ORN], [0.0, 0.0, 0.4], [0.032], TH, MARGIN)

    assert lift == [True, False]
    assert len(pos) == 2
    # 抬升点: 目标正上方, 高度按实测最低点反推 -> 最低点正好落到 z_min_threshold + margin
    assert pos[0][:2] == [0.5, 0.0]
    assert pos[0][2] == pytest.approx(0.05 + (TH - 0.032) + MARGIN)
    assert pos[1] == [0.5, 0.0, 0.05]
    assert orn[0] == ORN and orn[1] == ORN


def test_pure_vertical_descent_is_not_lifted():
    """上一个点已经在目标正上方 -> 直接下探, 抓取/放桌面要靠这个贴到桌面。"""
    pos, orn, lift = insert_lift_waypoints(
        [[0.5, 0.0, 0.05]], [ORN], [0.5, 0.0, 0.4], [0.032], TH, MARGIN)

    assert lift == [False]
    assert pos == [[0.5, 0.0, 0.05]]


def test_high_target_is_not_lifted():
    pos, orn, lift = insert_lift_waypoints(
        [[0.5, 0.0, 0.3]], [ORN], [0.0, 0.0, 0.4], [0.136], TH, MARGIN)

    assert lift == [False]
    assert pos == [[0.5, 0.0, 0.3]]


def test_second_low_target_is_measured_against_the_first():
    """ref_pos 要跟着路点走, 否则第二个低目标会被当成"没横向位移"而漏掉抬升。"""
    pos, orn, lift = insert_lift_waypoints(
        [[0.5, 0.0, 0.05], [0.9, 0.0, 0.05]], [ORN, ORN],
        [0.0, 0.0, 0.4], [0.032, 0.032], TH, MARGIN)

    assert lift == [True, False, True, False]
    assert [p[:2] for p in pos] == [[0.5, 0.0], [0.5, 0.0], [0.9, 0.0], [0.9, 0.0]]
    assert pos[2][2] == pytest.approx(0.05 + (TH - 0.032) + MARGIN)
    # 第二个目标是从第一个抬升点横向过去的, 高度按它自己的最低点算
    assert pos[3] == [0.9, 0.0, 0.05]


def test_already_safe_target_is_untouched():
    """最低点已经高于横向安全高度 -> 一点不动。"""
    pos, orn, lift = insert_lift_waypoints(
        [[0.5, 0.0, 0.2]], [ORN], [0.0, 0.0, 0.4], [0.15], TH, MARGIN)

    assert lift == [False]
    assert pos == [[0.5, 0.0, 0.2]]


def test_pregrasp_offset_is_not_treated_as_a_lateral_sweep():
    """预抓取位姿 -> 抓取位姿只差沿夹爪 z 的 2cm 偏移, 夹爪一斜 XY 就差 ~1.4cm。

    实测 (2026-10-04) 一次橙子抓取: 为了跨这 1.4cm, 臂先抬 107mm 再落回来 ——
    就是用户看到的"上下来回一段才抓"。这种短位移不构成横扫, 不该补抬升路点。
    """
    orn = [0.0, 0.0, 0.0, 1.0]
    prep = [0.30, -0.10, 0.16]        # 预抓取: 贴桌面
    grasp = [0.2886, -0.1132, 0.158]  # 抓取: 与预抓取 XY 差 1.4cm
    pos, _, lift = insert_lift_waypoints(
        [prep, grasp], [orn, orn], [0.30, -0.10, 0.37], [0.004, 0.004], TH, MARGIN,
        lateral_threshold=0.05)

    assert lift == [False, False], "1.4cm 的位移不该被当成横向移动"
    assert pos == [prep, grasp], "位姿本身不该被改动"


def test_a_real_sweep_still_gets_its_lift():
    """长距离横向过去 + 贴桌面, 仍然要抬 —— 修完不能把保护一起去掉。"""
    orn = [0.0, 0.0, 0.0, 1.0]
    pos, _, lift = insert_lift_waypoints(
        [[0.5, 0.0, 0.05]], [orn], [0.0, 0.0, 0.4], [0.032], TH, MARGIN,
        lateral_threshold=0.05)

    assert lift == [True, False]
    assert pos[0][2] > pos[1][2], "抬升路点应在目标正上方"
