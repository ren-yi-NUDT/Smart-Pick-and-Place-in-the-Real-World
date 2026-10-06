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
# 候选重试: 最竖直的先试, 上限 MAX_GRASP_CANDIDATES 个
# ---------------------------------------------------------------------------
def test_grasp_retries_candidates_but_is_capped():
    """一个位姿上试到夹住为止, 但不超过上限。

    原版 planner.py 是"每个候选都试", 0f0b421 砍成只试一个 —— 砍掉的那半边正是抓取
    成功率。这里钉住三件事: 还是只有一个执行点 (不再按分数逐个试到十几个), 有上限,
    且挑中的那个排第一。
    """
    from skills.pick_and_place import MAX_GRASP_CANDIDATES

    src = (ROOT / "skills/pick_and_place.py").read_text()
    assert src.count("self._execute_grasping_twin_js_2(") == 1
    assert "for i in range(len(grasping_pose_world))" not in src
    assert "Checking pose:" not in src
    assert "order[:MAX_GRASP_CANDIDATES]" in src
    assert 1 < MAX_GRASP_CANDIDATES <= 5        # 1 = 回到只试一个; 太大 = 反复开合


def test_a_successful_grasp_stops_walking_the_observation_poses(monkeypatch):
    """夹住了就走人, 不去下一个观察位姿。

    2026-10-06 抓黄瓜: 地图 seen_from=['grasp2','grasp3'], 在 grasp2 上第一个候选就
    夹住了 (soft_close 报 object_detected), 但内层的 break 只跳出**候选**循环, 外层
    继续走到 grasp3 —— 手里攥着黄瓜, 那儿当然检不出目标, 于是报"东西被挪了"停下来
    问人, 整条流水线假失败, 手里的果子还得人工放。
    """
    import numpy as np

    s = _skill()
    s.config = types.SimpleNamespace(
        default_traj_js={k: {"J1": 0.0} for k in POSES},
        get_pose=lambda name: {f"J{i}": 0.0 for i in range(1, 7)},
    )
    visited, grasped = [], []
    s.control_arm = lambda **kw: visited.append(kw.get("pose_type"))
    s.control_hand = lambda **kw: None
    s.get_camera_obs = lambda: (None, None)
    s.save_current_transformation = lambda: None
    s._perception = types.SimpleNamespace(detect_grasps=lambda *a, **k: [{}])
    s._filtering_pose = lambda *a, **k: ([np.eye(4)], True, [0.0])
    s._transform_anygrasp_pose = lambda *a, **k: [np.eye(4)]

    def fake_exec(c, idx=None):
        grasped.append(idx)
        s._obs_ee_world = [0.0] * 7        # 真实现里由 trajectory_ee[0] 填
        s._obs_js = [0.0] * 6
        return True

    s._execute_grasping_twin_js_2 = fake_exec
    s._twin = object()
    s.send_cmd_twin = lambda *a, **k: {"value": 1, "info": {
        "trajectory": [[0.0] * 6], "trajectory_ee": [[0.0] * 7]}}
    monkeypatch.setattr(scene_map, "load_fresh",
                        lambda *a, **k: _scene(_obj("cucumber", "green", ["grasp2", "grasp3"])))

    assert s.run(object="cucumber", container="desk") is True
    assert grasped == ["grasp2"], f"夹住之后还往下试: {grasped}"
    assert "grasp3" not in visited, visited


def test_grasp_logs_how_off_centre_the_candidate_is():
    """候选日志得报"手爪点离物体框中心多少像素"。

    用户 2026-10-06: "总感觉抓的很偏, 两次都是在边缘刚好"。_most_vertical 只看倾角,
    倾角跟"手爪点在不在物体中间"无关 —— 不报这个数就永远只能靠感觉。第一次实测:
    黄瓜第一个候选倾角 3.1° (非常正), 但离框中心 34px。
    """
    src = (ROOT / "skills/pick_and_place.py").read_text()
    assert "离框中心" in src
    assert "offsets_px" in src


def _desk_skill(send):
    import numpy as np  # noqa: F401
    s = _skill()
    s.config = types.SimpleNamespace(
        default_traj_js={k: {"J1": 0.0} for k in POSES},
        get_pose=lambda name: {f"J{i}": 90.0 for i in range(1, 7)},
    )
    s._obs_ee_world = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
    s._obs_js = [0.5] * 6
    s._twin = object()
    s.send_cmd_twin = send
    return s


def test_desk_transfer_is_twin_planned_from_where_the_arm_actually_is():
    """桌位姿的转移路径必须问过孪生, 而且起点是臂真正在的关节角。

    用户 2026-10-05: "上次重录后机械臂在位姿间转移时夹爪撞到桌面"。桌面模式原来发一个
    裸关节角, 臂自己在关节空间插值, 中间会不会扫到桌面没人知道。

    两问: ① 桌位姿 -> 观察位姿 (既是判据, 又顺手把桌位姿的 FK 换出来 —— 孪生的
    trajectory_ee[0] 就是 current_js 的 FK, 它没有单独的 FK 接口); ② 从观察位姿的
    **录下来的**关节角规划过去。第 ② 问不能用第 ① 问的轨迹倒过来顶替: 那条轨迹的
    起点是孪生对观察位姿的 IK 解, 实测跟录下来的差 J4 12°、J6 10°。
    """
    import numpy as np
    calls = []

    def fake_send(twin, data):
        calls.append(dict(data["cnfg"]))
        n = len(calls)
        return {"value": 1, "info": {"trajectory": [[float(n)] * 6],
                                     "trajectory_ee": [[9.0, 9.0, 9.0, 0, 0, 0, 1]]}}

    traj = _desk_skill(fake_send)._desk_transfer("desk_pose_1")
    assert len(calls) == 2, "得问两次"
    assert calls[0]["target_pose"] == [[1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]], \
        "第①问的目标是观察位姿的末端位姿"
    assert abs(calls[0]["current_js"][0] - np.pi / 2) < 1e-9, \
        "第①问的起点是桌位姿的关节角 (弧度)"
    assert calls[1]["current_js"] == [0.5] * 6, "第②问必须从录下来的观察位姿关节角出发"
    assert calls[1]["target_pose"] == [[9.0, 9.0, 9.0, 0, 0, 0, 1]], \
        "第②问的目标是第①问换出来的桌位姿 FK"
    assert abs(traj[0][0] - np.degrees(2.0)) < 1e-9, "执行的得是第②问那条轨迹, 换成度"


def test_desk_transfer_refuses_rather_than_guessing():
    """孪生不给规划 / 拿不到观察位姿的末端位姿或关节角, 都返回 None —— 宁可不放, 不猜。"""
    s = _desk_skill(lambda *a, **k: {"value": 0, "info": {}})
    assert s._desk_transfer("desk_pose_1") is None
    for missing in ("_obs_ee_world", "_obs_js"):
        s = _desk_skill(lambda *a, **k: {"value": 1, "info": {}})
        setattr(s, missing, None)
        assert s._desk_transfer("desk_pose_1") is None, missing


def test_pre_grasp_falls_back_to_a_lower_lift():
    """预抓取默认抬 4cm, 够不着时降到 1cm。

    link_6 比指尖高 172mm, 所以预抓取抬多高, 腕部半径就多出去多少。2026-10-06 桃子:
    指尖半径 0.439m, 抬 4cm 时腕部 0.637m > 实测 0.62m 上限 → 孪生判不可达, 而同一个
    候选的执行位姿只有 0.609m, 残差 0.3mm。降到 1cm 后腕部 0.615m, 进得去。
    """
    import numpy as np
    from skills.pick_and_place import PRE_GRASP_OFFSETS, _pre_grasp_pose

    assert PRE_GRASP_OFFSETS[0] == (0.02, 0.02), "默认档不能动: 4cm 是给飞行段留的余量"

    grasp = np.eye(4)
    grasp[:3, :3] = np.diag([1.0, -1.0, -1.0])      # 接近轴朝下 (竖直下爪)
    grasp[:3, 3] = [-0.3965, 0.1872, 0.2493]        # 桃子那次的执行位姿
    lift = lambda off: _pre_grasp_pose(grasp, *off, np.eye(4))[2, 3] - grasp[2, 3]
    assert abs(lift(PRE_GRASP_OFFSETS[0]) - 0.04) < 1e-9

    r_xy = float(np.hypot(-0.3965, 0.1872))
    wrist_r = lambda off: float(np.hypot(r_xy, lift(off) + 0.2493 + 0.172))
    assert wrist_r(PRE_GRASP_OFFSETS[0]) > 0.62, "默认档在桃子这儿就是够不着"
    assert wrist_r(PRE_GRASP_OFFSETS[-1]) <= 0.62, "兜底档得真进得去"


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
