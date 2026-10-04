#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""位姿表必须整体换到腕部有余量的 ZYZ 等价解, 否则抓取候选会被孪生判成不可达。

背景 (2026-10-04 实测): 录下来的位姿 J4 ≈ -163.8~-172.5°, 离真机 ±177.96° 限位只剩
~9°。腕部没有余量时, 任何需要改变夹爪朝向的抓取候选都会把 IK 逼到 ZYZ 翻转分支;
而翻转分支要 J4 从 -169° 走到 +172°, 只能穿过 ±180° 的限位死区 —— 走不过去, 孪生的
BRANCH_JUMP 保护因此正确地把候选判成不可用 (dependence/twin_inference/twin.py:530)。

实测同一个橙子同一个位姿: 原始 grasp1 (J4=-169°) 7 个候选只有 2 个能生成轨迹;
换成翻转解 (J4=+11°) 10 个里 9 个能。

**必须整表一起翻**: 翻转是等距变换, 全表一起翻则两两相对距离逐个不变 (一秒都不多走),
而只翻一部分会在两拨位姿之间造出 179° 的腕部大回转。
"""

import itertools
import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from core.config import (Config, LIMITS, fold, within_limits,       # noqa: E402
                         wrist_headroom_pose)

# 磁盘上的原始值 (未被翻转的)
RAW = json.load(open(os.path.join(ROOT, "robot_config.json")))
POSES = {k: v for k, v in RAW.get("default_traj_js", {}).items()
         if isinstance(v, dict) and "J4" in v}
POSES.update({k: v for k, v in RAW.items()
              if isinstance(v, dict) and "J1" in v})

J4_EDGES = [0.0, 90.0, -90.0, 90.5, -90.5, 177.0, -177.0,
            177.96, -177.96, 179.0, -179.0, 180.0, -180.0]


# ---------------------------------------------------------------------------
# fold: 纯算术, 不认识限位
# ---------------------------------------------------------------------------
def test_fold_is_pure_arithmetic():
    """折角不因"这个角越限"而改成别的 —— 那是 within_limits 的事。"""
    assert fold(-179.5, 360.0) == -179.5        # 落在 J4 死区里, 但 fold 不管
    assert fold(179.5, 360.0) == 179.5
    assert fold(180.0, 360.0) == 180.0          # 值域是 (-180, 180]
    assert fold(-180.0, 360.0) == 180.0
    assert fold(359.0, 360.0) == -1.0
    assert fold(-359.0, 360.0) == 1.0
    assert fold(0.0, 360.0) == 0.0


def test_within_limits_flags_the_j4_dead_zone():
    """J4 行程 355.92°, ±180 附近有 4° 不可达缺口。"""
    assert not within_limits({"J4": -179.5})
    assert not within_limits({"J4": 179.5})
    assert within_limits({"J4": 177.9})
    assert within_limits({"J4": -177.9})


def test_within_limits_knows_j6_is_wider():
    """J6 限位 ±359.82, 比整圈宽 —— 折 360 的结果域 (-180,180] 恒在限位内。"""
    assert not within_limits({"J6": 380.0})
    assert within_limits({"J6": 359.8})
    assert within_limits({"J6": fold(200.0 + 180.0, 360.0)})


# ---------------------------------------------------------------------------
# 幂等: reload 反复调也必须一致
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("name", sorted(POSES))
def test_flip_is_idempotent(name):
    once = wrist_headroom_pose(POSES[name])
    assert wrist_headroom_pose(once) == once


@pytest.mark.parametrize("j4", J4_EDGES)
def test_flip_is_idempotent_on_edges(j4):
    p = {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": j4, "J5": 30.0, "J6": 200.0}
    once = wrist_headroom_pose(p)
    assert wrist_headroom_pose(once) == once


# ---------------------------------------------------------------------------
# 翻转的性质
# ---------------------------------------------------------------------------
def test_grasp1_flip_is_exact():
    """翻转就是 (J4+180, -J5, J6+180), J1-J3 逐字不动。"""
    got = wrist_headroom_pose(POSES["grasp1"])
    assert got["J1"] == POSES["grasp1"]["J1"]
    assert got["J2"] == POSES["grasp1"]["J2"]
    assert got["J3"] == POSES["grasp1"]["J3"]
    assert got["J4"] == pytest.approx(11.028)
    assert got["J5"] == pytest.approx(69.998)
    assert got["J6"] == pytest.approx(158.8)


@pytest.mark.parametrize("name", sorted(POSES))
def test_flip_gives_wrist_headroom(name):
    """翻转后 J4 必须离限位足够远, 这才是修的东西。"""
    got = wrist_headroom_pose(POSES[name])
    assert abs(got["J4"]) <= 90.0, f"{name}: 翻转后 J4={got['J4']} 还是贴着限位"
    assert LIMITS["J4"] - abs(got["J4"]) > 80.0, f"{name}: 腕部余量不足"


@pytest.mark.parametrize("name", sorted(POSES))
def test_flip_stays_inside_real_limits(name):
    """等价解也必须落在真机限位里, 否则真机执行不了。"""
    assert within_limits(wrist_headroom_pose(POSES[name]))


def test_flip_preserves_every_pairwise_wrist_distance():
    """全表一起翻 = 等距重标号, 所以一秒都不多走。这是"整表一起翻"的全部理由。

    J4/J6 的差逐字不变, J5 的差只变号 —— 所以比的是绝对值。
    """
    for a, b in itertools.combinations(sorted(POSES), 2):
        pa, pb = POSES[a], POSES[b]
        fa, fb = wrist_headroom_pose(pa), wrist_headroom_pose(pb)
        for j in ("J4", "J5", "J6"):
            before = abs(fold(pa[j] - pb[j]))
            after = abs(fold(fa[j] - fb[j]))
            assert abs(after - before) < 1e-9, f"{a}/{b} 的 {j} 距离变了: {before} -> {after}"


# ---------------------------------------------------------------------------
# Config 加载
# ---------------------------------------------------------------------------
def test_config_flips_the_whole_table():
    """递送/扔垃圾/放桌面的位姿也要翻 —— 留一半在原支路就会造出 179° 的腕部大回转。"""
    cfg = Config()
    for name in POSES:
        got = cfg.get_pose(name)
        assert got is not None, f"{name} 查不到"
        assert abs(got["J4"]) <= 90.0, f"{name} 加载后 J4={got['J4']} 没换支路"


def test_config_actually_flips_top_level_poses():
    """顶层位姿(递送等)以前是不翻的, 现在必须翻 —— 这条是那个行为的反向断言。"""
    cfg = Config()
    assert cfg.get_pose("handover_pose") != RAW["handover_pose"]
    assert cfg.get_pose("handover_pose") == wrist_headroom_pose(RAW["handover_pose"])


def test_config_leaves_the_file_on_disk_alone():
    """翻转只在内存里做, 录下来的原始值要原样留着。"""
    path = os.path.join(ROOT, "robot_config.json")
    before = open(path).read()
    Config()
    assert open(path).read() == before
