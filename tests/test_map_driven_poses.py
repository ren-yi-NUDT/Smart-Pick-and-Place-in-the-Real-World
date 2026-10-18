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
# _poses_for: 地图命中就去看见过它的那些位姿, 按 seen_from 的顺序
# ---------------------------------------------------------------------------
def test_map_hit_goes_to_every_pose_that_saw_it_first_one_first():
    """seen_from 的顺序就是"最先看见它"的顺序 —— 按顺序试, 第一个能用就停。

    不能只去第一个: "看得见"不等于"抓得着"。2026-10-06 桃子地图命中 grasp2, YOLO
    在那儿确实检出桃子, 但 AnyGrasp 的候选全落在旁边的黄瓜上, 一个都不在桃子上。
    """
    s = _skill()
    poses, from_map = s._poses_for(_scene(_obj("plate", "blue", ["grasp4", "grasp2"])),
                                   "blue plate", "容器")
    assert poses == ["grasp4", "grasp2"], poses
    assert from_map is True


def test_map_hit_keeps_every_pose_that_saw_it():
    s = _skill()
    poses, _ = s._poses_for(_scene(_obj("plate", "blue", POSES)), "blue plate", "容器")
    assert poses == POSES, poses


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
    """反向断言: 地图在的时候必须真的往下走, 否则上面那条测试只是"永远返回 False"。

    抓取阶段复刻原项目管线 (core/grasp_pipeline.py) 后, 观察位姿按配置的 grasp1-4
    走, 地图只管"放" —— 这里断言 run() 确实把物体交给了抓取阶段。
    """
    s = _skill()
    grasped = []
    s._grasp_pipeline = types.SimpleNamespace(
        visual_grasp=lambda obj: grasped.append(obj) or False)
    monkeypatch.setattr(scene_map, "load_fresh",
                        lambda *a, **k: _scene(_obj("plate", "blue", ["grasp4"])))
    assert s.run(object="plum", container="blue plate") is False   # 没抓到, 但走过了
    assert grasped == ["plum"]


def test_placement_tilts_pull_the_wrist_back_toward_the_base():
    """容器远的时候竖直末端朝向够不着 —— 倾倒把腕部沿容器方向收回来。

    腕部比指尖远 172mm (实测: 末端竖直时 link_6 正好在指尖上方 172mm), 所以竖直
    朝下 = 把腕部往外多推 172mm。2026-10-06 蓝盘子 xy 半径 0.587: 竖直 delta 0.165
    不可达, 朝背离容器一侧倒 45° 后 delta 0.0001 精确命中。
    """
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    from skills.pick_and_place import PLACEMENT_TILTS, _placement_orn_candidates

    base = R.from_euler("z", 30, degrees=True).as_matrix()   # 容器自身朝向, 随便一个
    plate_xy = np.array([-0.364, 0.460])                     # 蓝盘子

    cands = _placement_orn_candidates(base, plate_xy)
    assert len(cands) == 1 + len(PLACEMENT_TILTS)
    assert np.allclose(cands[0], base), "第一档必须还是原样, 近处容器行为不能变"
    assert all(c[2, 2] < -0.4 for c in cands[1:]), "倾倒档的接近轴得朝下"

    offset = np.array([0.0, 0.0, -0.172])                    # 指尖 -> link_6
    wrist_xy = lambda rot: np.linalg.norm((plate_xy + (rot @ offset)[:2]))
    assert abs(wrist_xy(np.diag([1.0, -1.0, -1.0])) - 0.587) < 0.01   # 竖直时 0.587
    assert wrist_xy(cands[-1]) < 0.50, f"倒 60° 没把腕部收回来: {wrist_xy(cands[-1]):.4f}"
