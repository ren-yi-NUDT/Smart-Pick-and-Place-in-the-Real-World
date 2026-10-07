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
def test_grasp_tries_exactly_one_candidate_per_pose():
    """一个观察位姿上只试一个候选, 夹不到就换位姿。

    用户 2026-10-06 / 2026-10-07: "一次抓不到直接继续, 不要再跑候选"。中间有一版做成
    "试到夹住为止, 上限 3", 理由是候选之间的差别真实存在 —— 但现场看到的是夹爪在果子
    旁边反复开合, 而"哪个好"没有可靠的事前判据。钉死: 一个执行点, 不按分数遍历。
    """
    from skills.pick_and_place import MAX_GRASP_CANDIDATES

    assert MAX_GRASP_CANDIDATES == 1
    src = (ROOT / "skills/pick_and_place.py").read_text()
    assert src.count("self._execute_grasping_twin_js_2(") == 1
    assert "for i in range(len(grasping_pose_world))" not in src
    assert "Checking pose:" not in src
    assert "order[:MAX_GRASP_CANDIDATES]" not in src, "别再按候选循环了"


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
    s.T_hand_effector_to_arm_endlink = np.eye(4)   # _plan_grasp 用它算末端目标
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


def test_grasp_skips_candidates_the_twin_cannot_reach():
    """下爪要正, 但**够得着**优先。

    2026-10-07 火龙果: 最竖直的候选倾角只有 3°, 而 r≈0.45 上工具竖直意味着 link_6
    还要沿工具轴往外 172mm —— 孪生报 delta_xyz=24mm 够不着, 一次抓取都没发生就
    "抓取失败"。同一个果子上倾角 24~49° 的候选孪生全都能规划 (实测 7/10 可达)。
    竖直不是目的, 够得着才是。
    """
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

    s = PickAndPlaceSkill.__new__(PickAndPlaceSkill)
    # 孪生只认倾角 >= 20° 的候选: 更竖直的 link_6 探得太远, 够不着
    s._plan_grasp = lambda mat, idx: {"value": int(PickAndPlaceSkill._tilt_deg(mat) >= 20.0)}

    # 列表顺序 = AnyGrasp 分数从高到低。最竖直的 3.1° 够不着, 该退到 24.3°
    candidates = [pose(38.6), pose(3.1), pose(51.1), pose(24.3)]
    got = s._pick_reachable(candidates, "grasp1")
    assert abs(PickAndPlaceSkill._tilt_deg(got) - 24.3) < 0.1, "没跳过够不着的那个"

    # 一个都够不着 -> None, 上层换下一个观察位姿 (而不是硬发一条到不了的轨迹)
    assert s._pick_reachable([pose(3.1), pose(5.0)], "grasp1") is None

    # 全是斜的时候挑最正的 —— 不做硬性角度门槛
    assert abs(PickAndPlaceSkill._tilt_deg(
        s._pick_reachable([pose(70.0), pose(55.0)], "grasp1")) - 55.0) < 0.1


def test_grasp_height_lands_on_the_widest_cross_section():
    """指尖落在"接触点到桌面高度的一半" = 物体的最宽处。

    2026-10-07 火龙果 (高 8.6cm): 桌面 z=0.009, AnyGrasp 接触点在果子顶 z=0.092,
    手指就停在果子顶上合拢, 现场"夹爪空合"。果子高度中点是 0.052。
    """
    from core.config import TABLE_Z_M
    from skills.pick_and_place import _grasp_height

    assert abs(_grasp_height(0.092) - 0.0505) < 0.002, "没落到火龙果的高度中点"
    # 躺着的东西: 顶面高度的一半 = 轴心高度 (黄瓜直径 4.4cm -> 轴心在桌面之上 2.2cm)
    assert abs(_grasp_height(TABLE_Z_M + 0.044) - (TABLE_Z_M + 0.022)) < 1e-9
    # 只会往下调: 接触点永远在物体表面, 表面不会低于最宽处
    assert _grasp_height(0.30) < 0.30


def test_grasp_prefers_a_centred_candidate_over_a_vertical_one():
    """抓准优先于抓正。

    2026-10-07 用户: "都抓的偏前"。实测火龙果 grasp3 那张候选表上, 最竖直的候选离
    物体框中心 89px (快到框边了), 而最准的只有 12px —— 只看倾角就会挑中那个偏的。
    """
    import numpy as np
    from skills.pick_and_place import PickAndPlaceSkill

    def pose(tilt_deg):
        t = np.radians(tilt_deg)
        T = np.eye(4)
        T[:3, :3] = np.array([[np.cos(t), 0, np.sin(t)],
                              [0, 1, 0],
                              [-np.sin(t), 0, np.cos(t)]])
        return T

    s = PickAndPlaceSkill.__new__(PickAndPlaceSkill)
    s._plan_grasp = lambda mat, idx: {"value": 1}
    # (倾角, 离框中心 px) 取自实测: 最竖直的 24.3° 在 89px 处, 43.2° 那个在 12px 处
    cands, offs = [pose(24.3), pose(49.4), pose(43.2)], [89.0, 40.0, 12.0]
    got = s._pick_reachable(cands, "grasp1", offs)
    assert abs(PickAndPlaceSkill._tilt_deg(got) - 43.2) < 0.1, "挑了偏的那个"

    # 最准的那个够不着 -> 放宽到全部候选, 不能因为"最准的够不着"就整个放弃
    s._plan_grasp = lambda mat, idx: {"value": int(PickAndPlaceSkill._tilt_deg(mat) < 30)}
    got = s._pick_reachable(cands, "grasp1", offs)
    assert abs(PickAndPlaceSkill._tilt_deg(got) - 24.3) < 0.1, "没放宽到全部"
