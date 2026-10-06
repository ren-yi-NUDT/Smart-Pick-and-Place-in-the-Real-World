#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图决定去哪个观察位姿, 现场检测只负责否决。

背景 (用户 2026-10-04): "固定一开始 lookaround 一遍, 然后后面直接抓直接放" ——
以前抓和放各自把 grasp1-4 走一遍, 走完才发现容器在背后, 于是再走一遍。现在地图说
"蓝盘子 grasp4 看得见" 就直接去 grasp4, 只去一个。

配套的三条:
  - 地图过期/没有 → 停下让人重扫, 不退回四位姿瞎找 (退回等于把"该重扫了"吃掉);
  - 地图说看得见、现场认不出 → 停下问人 (东西被挪了还是地图过期了, 只有人知道);
  - 一次抓不到就认, 不堆候选。
"""

import os
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from core import scene as scene_map                                    # noqa: E402
from skills.pick_and_place import PickAndPlaceSkill                    # noqa: E402

POSES = ["grasp1", "grasp2", "grasp3", "grasp4"]


def _skill():
    """不碰 config/硬件的空壳 —— 这些方法只用得到 config.default_traj_js。"""
    s = PickAndPlaceSkill.__new__(PickAndPlaceSkill)
    s.config = types.SimpleNamespace(default_traj_js={k: {"J1": 0.0} for k in POSES})
    s._perception = None       # 属性是只读的, 只能填后备字段
    s._hand = None
    s.save_path = "./log"
    s._camera = None
    return s


def _scene(*objects):
    return {"scanned_at": "2026-10-04 12:00:00", "objects": list(objects)}


def _obj(name, color, seen_from, conf=0.9):
    return {"name": name, "color": color, "world": [0.0, 0.0, 0.0],
            "conf": conf, "seen_from": list(seen_from)}


# ---------------------------------------------------------------------------
# _poses_for: 地图命中只去一个位姿
# ---------------------------------------------------------------------------
def test_map_hit_goes_to_the_first_pose_that_saw_it():
    """seen_from 的顺序就是"最先看见它"的顺序 —— 只去第一个, 走路最少。"""
    s = _skill()
    poses, from_map = s._poses_for(_scene(_obj("plate", "blue", ["grasp4", "grasp2"])),
                                   "blue plate", "容器")
    assert poses == ["grasp4"], poses
    assert from_map is True


def test_map_hit_keeps_only_one_pose_even_with_many():
    s = _skill()
    poses, _ = s._poses_for(_scene(_obj("plate", "blue", POSES)), "blue plate", "容器")
    assert poses == ["grasp1"], poses


def test_color_decides_between_two_plates():
    """两个盘子同名不同色, 说"蓝盘子"就不能拿到粉的那个。"""
    s = _skill()
    scene = _scene(_obj("plate", "pink", ["grasp1"], conf=0.95),
                   _obj("plate", "blue", ["grasp4"], conf=0.60))
    assert s._poses_for(scene, "blue plate", "容器")[0] == ["grasp4"]
    assert s._poses_for(scene, "pink plate", "容器")[0] == ["grasp1"]


def test_map_miss_falls_back_to_every_pose():
    """地图里没有就老实回退 —— 但把话说清楚 (日志里的那条黄色告警)。"""
    s = _skill()
    poses, from_map = s._poses_for(_scene(_obj("plate", "blue", ["grasp4"])),
                                   "plum", "物体")
    assert poses == POSES and from_map is False


def test_stale_pose_name_falls_back():
    """地图是旧版本录的、位姿名对不上时不能返回一个不存在的位姿。"""
    s = _skill()
    poses, from_map = s._poses_for(_scene(_obj("plate", "blue", ["grasp9"])),
                                   "blue plate", "容器")
    assert poses == POSES and from_map is False


def test_colored_phrase_that_yolo_cannot_split_still_looks_up():
    """"red fruit" 拆不开, 整串当名字查 —— 地图里若真有这个名字也得命中。"""
    s = _skill()
    poses, from_map = s._poses_for(_scene(_obj("red fruit", None, ["grasp3"])),
                                   "red fruit", "物体")
    assert poses == ["grasp3"] and from_map is True


# ---------------------------------------------------------------------------
# _map_still_holds: 现场核对
# ---------------------------------------------------------------------------
def _detections(*pairs):
    """detect_scene 的返回格式: (名字, 颜色, 框, 置信度) 四元组。"""
    return [(n, c, [0, 0, 1, 1], 0.5) for n, c in pairs]


@pytest.fixture
def fake_detect(monkeypatch):
    """替掉建图那套检测, 让核对逻辑可以离线跑。"""
    def install(rows):
        monkeypatch.setattr(scene_map, "detect_scene",
                            lambda perc, rgb, conf=None: _detections(*rows))
    return install


def test_map_still_holds_when_color_matches(fake_detect):
    fake_detect([("plate", "blue")])
    assert _skill()._map_still_holds(None, "blue plate", None) is True


def test_map_still_holds_rejects_the_other_plate(fake_detect):
    """现场只看见粉盘子 —— 地图说的蓝盘子就是不见了, 该停。"""
    fake_detect([("plate", "pink")])
    assert _skill()._map_still_holds(None, "blue plate", None) is False


def test_map_still_holds_ignores_color_when_phrase_has_none(fake_detect):
    fake_detect([("plate", "pink")])
    assert _skill()._map_still_holds(None, "plate", None) is True


def test_map_still_holds_rejects_a_different_object(fake_detect):
    fake_detect([("cucumber", "green")])
    assert _skill()._map_still_holds(None, "plum", None) is False


# ---------------------------------------------------------------------------
# run(): 地图过期就停, 一步都不动
# ---------------------------------------------------------------------------
def test_no_map_means_no_motion(monkeypatch):
    """过期地图比没有地图更糟 —— 照着旧坐标去够会够到空处。所以直接停。

    load_fresh 对"没扫过 / 文件坏了 / 过期"三种情况一律返回 None, 所以这里只模拟它。
    """
    s = _skill()
    moved = []
    s.control_arm = lambda **kw: moved.append(kw)
    monkeypatch.setattr(scene_map, "load_fresh", lambda *a, **k: None)
    assert s.run(object="plum", container="blue plate") is False
    assert moved == []


def test_a_fresh_map_lets_it_move(monkeypatch):
    """反向断言: 地图在的时候必须真的往下走, 否则上面那条测试只是"永远返回 False"。"""
    s = _skill()
    moved = []
    s.control_arm = lambda **kw: moved.append(kw.get("pose_type"))
    s.control_hand = lambda **kw: None
    s.get_camera_obs = lambda: (None, None)
    s.save_current_transformation = lambda: None
    s._perception = types.SimpleNamespace(detect_grasps=lambda *a, **k: [])
    monkeypatch.setattr(scene_map, "load_fresh",
                        lambda *a, **k: _scene(_obj("plate", "blue", ["grasp4"])))
    assert s.run(object="plum", container="blue plate") is False   # 没抓到, 但走过了
    assert moved == ["grasp1"] + POSES                             # 物体不在图上 → 逐个看


# ---------------------------------------------------------------------------
# 只试一个候选
# ---------------------------------------------------------------------------
def test_grasp_tries_exactly_one_candidate():
    """一次抓不到就认。堆候选的现场观感是"夹爪在果子旁边反复开合"。"""
    src = (ROOT / "skills/pick_and_place.py").read_text()
    # 只执行一次。具体挑哪个由 _most_vertical 决定 (2026-10-06 从"分数最高"改成
    # "最竖直"), 这里只管"只有一个候选被送去执行", 不把选择规则钉死在源码里。
    assert src.count("self._execute_grasping_twin_js_2(") == 1
    assert "for i in range(len(grasping_pose_world))" not in src
    assert "Checking pose:" not in src


def test_grasp_picks_most_vertical_candidate():
    """下爪要正。AnyGrasp 的分数衡量夹得稳不稳, 跟下爪正不正无关。"""
    import numpy as np
    from skills.pick_and_place import PickAndPlaceSkill

    def pose(tilt_deg):
        """绕 Y 轴转 tilt_deg 的位姿: 接近轴(第三列)离竖直 tilt_deg 度。"""
        t = np.radians(tilt_deg)
        T = np.eye(4)
        T[:3, :3] = np.array([[np.cos(t), 0, np.sin(t)],
                              [0, 1, 0],
                              [-np.sin(t), 0, np.cos(t)]])
        return T

    # 列表顺序 = AnyGrasp 分数从高到低; 最竖直的那个排在最后
    candidates = [pose(38.6), pose(24.3), pose(51.1), pose(14.6)]
    got = PickAndPlaceSkill._most_vertical(None, candidates)
    assert abs(PickAndPlaceSkill._tilt_deg(got) - 14.6) < 0.1
    # 全是斜的时候, 挑的还是里面最正的那个 —— 不做硬性角度门槛
    assert abs(PickAndPlaceSkill._tilt_deg(
        PickAndPlaceSkill._most_vertical(None, [pose(70.0), pose(55.0)])) - 55.0) < 0.1
