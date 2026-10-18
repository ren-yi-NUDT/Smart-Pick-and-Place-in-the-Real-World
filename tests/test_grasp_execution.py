"""复刻自原项目的抓取管线 (core/grasp_pipeline.py) 的离线回归。

不连接硬件: arm/hand/twin/perception 全部替换为记录调用的假件。
标定链 (T_base_to_cam / T_hand_to_end) 的组合顺序在这里逐条核对 —— 抓偏了先查这里。
"""
import math
import types

import numpy as np
import pytest

from core.grasp_pipeline import GraspPipeline

SCORING = {
    "weights": {"anygrasp": 0.4, "twin": 0.3, "width": 0.15, "length": 0.05,
                "angle": 0.1},
    "high_pregrasp_offset_m": 0.08,
    "pregrasp_offset_m": 0.02,
    "gripper_close_force": 10,
    "gripper_close_speed": 10,
    "gripper_hold_force": 40,
    "grasp_trajectory_speed": 6,
    "grasp_post_lift_speed": 15,
    "grasp_recovery_speed": 10,
    "verify_lift": True,
}


def make_pipeline(scoring=SCORING):
    """空壳 owner + 真管线。fake 属性按测试需要再补。"""
    owner = types.SimpleNamespace()
    owner.config = types.SimpleNamespace(
        shared={},
        get_grasp_scoring=lambda: dict(scoring),
        get_arm_config=lambda: {"twin_struct": "rm65"},
        default_traj_js={"grasp1": {f"J{i}": 20.0 for i in range(1, 7)}},
    )
    return GraspPipeline(owner)


# ---------------------------------------------------------------------------
# 速度换算 (原项目 test_grasp_pipeline_optimization 同款)
# ---------------------------------------------------------------------------
def test_grasp_arm_speed_defaults_to_double_and_can_be_overridden():
    pipeline = make_pipeline()
    scoring = {"grasp_trajectory_speed": 6}
    assert pipeline._grasp_arm_speed(scoring, "grasp_trajectory_speed", 12) == 12
    assert pipeline._grasp_arm_speed(scoring, "grasp_trajectory_speed", 12, 1.0) == 6


# ---------------------------------------------------------------------------
# 候选构建: 标定矩阵组合顺序 (抓偏先查这里)
# ---------------------------------------------------------------------------
def test_candidate_pose_composes_calibration_matrices_in_order():
    """T_world_hand = T_base_to_cam @ (T_grasp @ hand_convention), 末端 = 手位姿 @ T_hand_to_end。

    左臂 (Robotiq) 约定: hand_convention = self_rotation @ diag([-1,-1,1,1])。
    """
    from core.transforms import self_rotation_np

    pipeline = make_pipeline()
    pipeline._skill.T_base_to_cam = np.diag([1.0, 1.0, 1.0, 1.0])   # 单位阵便于手算
    pipeline._skill.T_hand_effector_to_arm_endlink = np.eye(4)
    pipeline._skill.T_hand_effector_to_arm_endlink[2, 3] = 0.172   # 指尖 -> link_6

    T_grasp = np.eye(4)
    T_grasp[:3, 3] = [0.30, 0.05, 0.20]
    raw = [{"trans": [0.30, 0.05, 0.20], "rotation_matrix": np.eye(3), "score": 0.9,
            "width": 0.05, "height": 0.03}]
    cands = pipeline._build_grasp_candidates(raw)
    assert len(cands) == 1
    c = cands[0]

    convention = self_rotation_np(np.array(
        [[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float))
    convention = convention @ np.diag([-1., -1., 1., 1.])
    expected_hand = np.eye(4) @ (T_grasp @ convention)
    if expected_hand[:3, 0][0] < 0:   # x 轴朝基座 -X 时翻转 (原版规则)
        expected_hand = expected_hand @ np.diag([-1., -1., 1., 1.])
    assert np.allclose(c["pose"], expected_hand)
    assert c["T_hand_to_end"][2, 3] == 0.172
    assert c["camera_translation_m"] == [0.30, 0.05, 0.20]


def test_candidate_scores_are_normalised_anygrasp_scores():
    pipeline = make_pipeline()
    pipeline._skill.T_base_to_cam = np.eye(4)
    pipeline._skill.T_hand_effector_to_arm_endlink = np.eye(4)
    raw = [{"trans": [0, 0, 0.2], "rotation_matrix": np.eye(3), "score": s,
            "width": 0.05, "height": 0.03} for s in (0.2, 0.8)]
    cands = pipeline._build_grasp_candidates(raw)
    assert cands[1]["anygrasp_normalized"] > cands[0]["anygrasp_normalized"]


# ---------------------------------------------------------------------------
# 目标位姿: 高/低预抓取 + 执行点三段, 偏置在 hand->end 之后按基座系加
# ---------------------------------------------------------------------------
def test_target_config_stages_three_waypoints_and_applies_base_offset():
    pipeline = make_pipeline({**SCORING, "execution_offset_base_m": [0.01, 0, 0]})
    pose = np.eye(4)
    pose[2, 3] = 0.20
    pose[:3, :3] = np.diag([1.0, -1.0, -1.0])       # 接近轴朝下
    pipeline._skill.T_hand_effector_to_arm_endlink = np.eye(4)
    cand = {"pose": pose, "T_hand_to_end": np.eye(4)}
    cfg = pipeline._grasp_target_config(cand, {f"J{i}": 20.0 for i in range(1, 7)})

    assert len(cfg["target_pose"]) == 3
    zs = [p[2] for p in cfg["target_pose"]]
    assert zs == pytest.approx([0.28, 0.22, 0.20])   # high(+0.08) / low(+0.02) / 执行
    xs = [p[0] for p in cfg["target_pose"]]
    assert xs == pytest.approx([0.01, 0.01, 0.01])   # 偏置三段都加 (基座系)
    assert cfg["current_js"] == pytest.approx([math.radians(20)] * 6)
    assert cfg["struct"] == "rm65"
    assert cfg["xyz_threshold"] == pytest.approx(0.015)   # shared 默认值
    assert cfg["rotation_interval_rad"] == pytest.approx(0.15)  # 服务端姿态加密步长
    assert cfg["joint_step_limit_rad"] == pytest.approx(0.12)   # 分支跳变时的关节插值步长


def test_plan_candidate_does_not_call_dead_generation3():
    """generation2 失败即放弃该候选。generation3 服务端从未接通 (不在
    SUPPORTED_TWIN_SRV_TYPE 里), 回退只会白问一轮。"""
    pipeline = make_pipeline()
    calls = []

    def gen2(cnfg):
        calls.append("gen2")
        return {"value": 0, "info": "unreachable"}

    pipeline._skill.twin = types.SimpleNamespace(generate_trajectory2=gen2)
    cand = {"pose": np.eye(4), "T_hand_to_end": np.eye(4)}
    assert not pipeline._plan_grasp_candidate(cand, {f"J{i}": 0.0 for i in range(1, 7)})
    assert set(calls) == {"gen2"}   # 各 high-pregrasp 档位都只问 gen2, 不回退 generation3
    assert cand["twin_reachable"] == 0.0


def test_planning_stops_at_highest_scoring_reachable_candidate():
    """按分数从高到低问孪生, 第一个可达的就停 —— 与全问一遍等价 (原版注释的理由)。"""
    pipeline = make_pipeline()
    candidates = [{"index": i} for i in range(3)]
    calls = []
    pipeline._score_grasp_candidates = lambda items: list(items)

    def plan(candidate, *args, **kwargs):
        calls.append(candidate["index"])
        candidate["twin_reachable"] = float(candidate["index"] == 1)
        return bool(candidate["twin_reachable"])

    pipeline._plan_grasp_candidate = plan
    selected = pipeline._plan_best_reachable_candidate(
        candidates, "grasp1", {f"J{i}": 0.0 for i in range(1, 7)})
    assert selected is candidates[1]
    assert calls == [0, 1]
    assert selected["observation_pose_name"] == "grasp1"


# ---------------------------------------------------------------------------
# 执行: 张爪确认 → 走轨迹 → soft close → hold 补夹 → 倒轨回撤 → 提升验证
# ---------------------------------------------------------------------------
class FakeHand:
    def __init__(self, detect=True):
        self.events = []
        self.detect = detect

    def is_fully_open(self):
        self.events.append("is_fully_open")
        return True

    def open(self):
        self.events.append("open")
        return {"value": True}

    def close(self, force=None, speed=None, soft=False):
        self.events.append(("close", force, soft))
        return {"object_detected": self.detect, "value": True}

    def hold(self, force=40):
        self.events.append(("hold", force))
        return {"value": True}

    def is_grasping(self, force=20):
        self.events.append(("is_grasping", force))
        return self.detect


class FakeArm:
    def __init__(self):
        self.trajs = []

    def execute_trajectory(self, trajectory, speed=20):
        self.trajs.append((np.asarray(trajectory).copy(), speed))
        return True

    def move_to_named_pose(self, pose, speed=30):
        self.events = getattr(self, "events", [])
        self.events.append(("recover", speed))
        return True


def _exec_pipeline(detect=True, scoring=SCORING):
    pipeline = make_pipeline(scoring)
    hand = FakeHand(detect=detect)
    arm = FakeArm()
    pipeline._skill.hand = hand
    pipeline._skill.arm = arm
    trajectory = [[0.0] * 6, [0.1] * 6, [0.2] * 6]
    cand = {"pose": np.eye(4), "T_hand_to_end": np.eye(4),
            "trajectory": trajectory, "composite_score": 0.9,
            "index": 0}
    return pipeline, cand, hand, arm


def test_successful_grasp_softcloses_holds_retraces_and_verifies_lift():
    pipeline, cand, hand, arm = _exec_pipeline()
    obs = {f"J{i}": 20.0 for i in range(1, 7)}
    assert pipeline._execute_scored_grasp(cand, obs) is True
    assert ("close", 10, True) in hand.events          # soft close, scoring 的力
    assert ("hold", 40) in hand.events                 # 补夹必须 hold, 不能 close 到底
    assert ("is_grasping", 10) in hand.events          # 提升后验证
    approach, approach_speed = arm.trajs[0]
    retreat, retreat_speed = arm.trajs[1]
    assert np.allclose(retreat, approach[::-1])        # 倒走同一条孪生轨迹
    assert retreat_speed == 15 * 2                     # grasp_post_lift_speed * scale


def test_failed_soft_close_recovers_to_observation_and_opens():
    pipeline, cand, hand, arm = _exec_pipeline(detect=False)
    obs = {f"J{i}": 20.0 for i in range(1, 7)}
    assert pipeline._execute_scored_grasp(cand, obs) is False
    assert arm.trajs and np.allclose(arm.trajs[-1][0], np.asarray(cand["trajectory"]) * 180 / np.pi)
    assert ("recover", 10 * 2) in arm.events           # 回观察位 (recovery speed * scale)
    assert "open" in hand.events
    assert pipeline._skill._last_grasp_failure["code"] == "GRASP_CONTACT_FAILED"


def test_gripper_not_open_aborts_before_any_motion():
    pipeline, cand, hand, arm = _exec_pipeline()
    hand.is_fully_open = lambda: False
    assert pipeline._execute_scored_grasp(cand, {f"J{i}": 20.0 for i in range(1, 7)}) is False
    assert arm.trajs == []
    assert pipeline._skill._last_grasp_failure["code"] == "GRASP_EXECUTION_FAILED"


# ---------------------------------------------------------------------------
# visual_grasp 端到端 (假件): 成功即停
# ---------------------------------------------------------------------------
def _visual_pipeline(detect=True):
    pipeline = make_pipeline()
    rgb = np.zeros((4, 4, 3), dtype=np.uint8)
    depth = np.zeros((4, 4), dtype=np.uint16)
    grasp = [{"trans": [0.30, 0.05, 0.20], "rotation_matrix": np.eye(3), "score": 0.9,
              "width": 0.05, "height": 0.03}]
    pipeline._skill.camera = types.SimpleNamespace(raw_depth=depth)
    pipeline._skill.get_camera_obs = lambda: (rgb, depth)
    pipeline._skill.save_current_transformation = lambda: None
    pipeline._skill.T_base_to_cam = np.eye(4)
    pipeline._skill.T_hand_effector_to_arm_endlink = np.eye(4)
    moved = []
    pipeline._skill.control_arm = lambda **kw: moved.append(kw.get("pose_type")) or True
    pipeline._skill.control_hand = lambda **kw: {"value": True}
    pipeline._skill.perception = types.SimpleNamespace(
        detect_grasps=lambda *a, **k: grasp if detect else [],
        filter_grasps_by_detection=lambda raw, rgb, **k: raw,
    )
    pipeline._skill.twin = types.SimpleNamespace(
        generate_trajectory2=lambda cnfg: {"value": 1, "info": {
            "trajectory": [[0.0] * 6], "trajectory_ee": [[0.1, 0.2, 0.3, 0, 0, 0, 1]]}},
        call_service=lambda srv, cnfg: {"value": 0, "info": {}},
    )
    pipeline._skill.hand = FakeHand(detect=detect)
    pipeline._skill.arm = FakeArm()
    return pipeline, moved


def test_visual_grasp_success_stops():
    pipeline, moved = _visual_pipeline()
    assert pipeline.visual_grasp("orange") is True
    assert moved == ["grasp1"]


def test_visual_grasp_no_candidates_reports_failure_code():
    pipeline, _ = _visual_pipeline(detect=False)
    assert pipeline.visual_grasp("orange") is False
    assert pipeline._skill._last_grasp_failure["code"] == "GRASP_NO_CANDIDATES"


# ---------------------------------------------------------------------------
# config API (抓取管线的配置入口)
# ---------------------------------------------------------------------------
def test_config_exposes_grasp_scoring_and_arm_config():
    from core.config import Config
    config = Config()
    scoring = config.get_grasp_scoring()
    assert scoring["weights"]["anygrasp"] == pytest.approx(0.4)
    assert scoring["gripper_hold_force"] == 40
    assert config.get_arm_config()["twin_struct"] == "rm65"
    assert config.sim_mode is False
    intr = config.get_camera_intrinsics()
    assert all(math.isfinite(v) for v in intr.values())
