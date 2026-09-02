from tools.dual_vlm_sorting import DualVlmSorter
from tools.dual_vlm_sorting import Observation
from skills.base import Skill
import numpy as np


def _actions():
    return [
        {"action_id": "left_1", "arm": "left"},
        {"action_id": "right_1", "arm": "right"},
        {"action_id": "left_2", "arm": "left"},
        {"action_id": "right_2", "arm": "right"},
    ]


def test_interleaved_sequence_has_home_barriers():
    assert DualVlmSorter._interleaved_execution_sequence(_actions()) == [
        {"phase": "pick_home", "action_id": "left_1"},
        {"phase": "pick_home", "action_id": "right_1"},
        {"phase": "place", "action_id": "left_1"},
        {"phase": "place", "action_id": "right_1"},
        {"phase": "pick_home", "action_id": "left_2"},
        {"phase": "pick_home", "action_id": "right_2"},
        {"phase": "place", "action_id": "left_2"},
        {"phase": "place", "action_id": "right_2"},
    ]


def test_interleaved_executor_calls_pick_home_before_each_pair_place():
    events = []

    class FakeSorter:
        def _execute_pick_then_home(self, action, helper):
            events.append(("pick_home", action["action_id"]))

        def _execute_place_phase(self, action, helper, destination_lock):
            events.append(("place", action["action_id"]))

    DualVlmSorter._execute_interleaved_home_pipeline(
        FakeSorter(),
        _actions(),
        {"left": object(), "right": object()},
    )
    assert events == [
        ("pick_home", "left_1"),
        ("pick_home", "right_1"),
        ("place", "left_1"),
        ("place", "right_1"),
        ("pick_home", "left_2"),
        ("pick_home", "right_2"),
        ("place", "left_2"),
        ("place", "right_2"),
    ]


def test_post_home_grasp_check_reenters_fresh_visual_grasp():
    events = []

    class FakeConfig:
        def get_pose(self, name, side=None):
            return {"J1": 0.0}

        def get_grasp_scoring(self, side):
            return {"gripper_close_force": 10}

    class FakeHand:
        def __init__(self):
            self.checks = 0

        def is_grasping(self, force=None):
            events.append(("verify", force))
            self.checks += 1
            return self.checks == 2

    class FakeHelper:
        config = FakeConfig()

        def __init__(self):
            self.hand = FakeHand()

        def gripper_for(self, side):
            return self.hand

    class FakeSorter:
        safety = {"max_post_home_regrasp_attempts": 2}
        plan = None

        def _execute_pick_phase(self, action, worker_helper, source_lock):
            events.append("visual_grasp")
            action["_candidate"] = {"trajectory": [[0.0, 0.0]]}

        def _move_named(self, side, pose, label, helper=None):
            events.append(("move", label))
            return True

        def _plan_place(self, side, candidate, destination, helper=None):
            events.append("plan_place")
            return [[1.0, 1.0]]

    action = {
        "action_id": "action_regrasp",
        "arm": "right",
        "_destination": object(),
    }
    helper = FakeHelper()

    DualVlmSorter._execute_pick_then_home(FakeSorter(), action, helper)

    assert events == [
        "visual_grasp",
        ("move", "home_after_grasp"),
        ("verify", 10),
        ("visual_grasp"),
        ("move", "home_after_grasp"),
        ("verify", 10),
        "plan_place",
    ]


def test_pipeline_finish_returns_home_then_opens_both_grippers():
    events = []

    class FakeConfig:
        def get_pose(self, name, side=None):
            return {"J1": 0.0}

    class FakeHand:
        def __init__(self, side):
            self.side = side

        def open(self):
            events.append(("open", self.side))

        def is_fully_open(self):
            return True

    class FakeHelper:
        def __init__(self, side):
            self.config = FakeConfig()
            self.hand = FakeHand(side)

        def gripper_for(self, side):
            return self.hand

    class FakeSorter:
        def _move_named(self, side, pose, label, helper=None):
            events.append(("home", side))
            return True

    helpers = {"left": FakeHelper("left"), "right": FakeHelper("right")}
    DualVlmSorter._finish_pipeline(FakeSorter(), helpers)
    assert events == [
        ("home", "left"),
        ("home", "right"),
        ("open", "left"),
        ("open", "right"),
    ]


def test_grasp_retry_uses_next_candidate_and_paired_place_trajectory():
    attempts = []

    class FakeConfig:
        sim_mode = False

    class FakeHelper:
        config = FakeConfig()

        def _execute_scored_grasp(self, candidate, side, obs_pose, hold_after_grasp):
            attempts.append((candidate["index"], side, hold_after_grasp))
            return len(attempts) == 2

    class FakeSorter:
        helper = FakeHelper()

        @staticmethod
        def _candidate_for_action(action, candidate=None):
            return candidate

    candidate_1 = {"index": 1, "trajectory": [0.0, 0.0]}
    candidate_2 = {"index": 2, "trajectory": [0.1, 0.2]}
    action = {
        "action_id": "action_1",
        "arm": "left",
        "_candidate": candidate_1,
        "_observation_pose": {"J1": 0.0},
        "place_trajectory_deg": [[1.0, 1.0]],
        "_grasp_options": [
            {"candidate": candidate_1, "place_trajectory": [[1.0, 1.0]]},
            {"candidate": candidate_2, "place_trajectory": [[2.0, 2.0]]},
        ],
    }

    assert DualVlmSorter._execute_grasp_with_retries(
        FakeSorter(), action, FakeHelper()
    ) is True
    assert attempts == [(1, "left", True), (2, "left", True)]
    assert action["_candidate"] is candidate_2
    assert action["place_trajectory_deg"] == [[2.0, 2.0]]


def test_pick_phase_delegates_to_fresh_single_arm_visual_grasp():
    events = []

    class FakeHelper:
        _last_successful_grasp_candidate = {
            "trajectory": [[0.0, 0.1]],
            "composite_score": 0.8,
            "camera_side": "right",
        }

        def visual_grasp(self, object_name, **kwargs):
            events.append((object_name, kwargs))
            return True

    class FakeSorter:
        safety = {}

        def _record_runtime_grasp(self, action, worker_helper):
            return DualVlmSorter._record_runtime_grasp(
                self, action, worker_helper
            )

    action = {
        "action_id": "action_orange",
        "object_name": "orange",
        "arm": "right",
        "source_id": "source_1",
    }
    source_lock = __import__("threading").Lock()

    DualVlmSorter._execute_pick_phase(
        FakeSorter(), action, FakeHelper(), source_lock
    )

    assert events == [
        (
            "orange",
            {
                "side": "right",
                "location": "desk_front",
                "hold_after_grasp": True,
                "use_vlm_grounding": True,
            },
        )
    ]
    assert action["_candidate"]["camera_side"] == "right"
    assert action["grasp_camera"] == "right"
    assert np.allclose(
        action["grasp_trajectory_deg"],
        [[0.0, 0.1 * 180.0 / np.pi]],
    )


def test_pick_phase_reinvokes_visual_grasp_after_failure():
    calls = []

    class FakeHelper:
        _last_successful_grasp_candidate = {
            "trajectory": [[0.0, 0.1]],
            "composite_score": 0.8,
            "camera_side": "left",
        }

        def visual_grasp(self, object_name, **kwargs):
            calls.append((object_name, kwargs))
            return len(calls) == 2

    class FakeSorter:
        safety = {"max_visual_grasp_attempts": 2}

        def _record_runtime_grasp(self, action, worker_helper):
            return DualVlmSorter._record_runtime_grasp(
                self, action, worker_helper
            )

    action = {
        "action_id": "action_moved_object",
        "object_name": "orange",
        "arm": "left",
        "source_id": "source_1",
    }
    source_lock = __import__("threading").Lock()

    DualVlmSorter._execute_pick_phase(
        FakeSorter(), action, FakeHelper(), source_lock
    )

    assert len(calls) == 2
    assert calls[0][1] == calls[1][1]
    assert calls[1][1]["use_vlm_grounding"] is True


def test_camera_base_transform_supports_both_camera_to_arm_directions():
    T_right_to_left = np.array([
        [0.0, -1.0, 0.0, 0.35],
        [1.0, 0.0, 0.0, -0.71],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

    class FakeSorter:
        def _calib_right_to_left(self):
            return T_right_to_left

    sorter = FakeSorter()
    assert np.allclose(
        DualVlmSorter._camera_base_to_arm(sorter, "right", "left"),
        T_right_to_left,
    )
    assert np.allclose(
        DualVlmSorter._camera_base_to_arm(sorter, "left", "right"),
        np.linalg.inv(T_right_to_left),
    )
    assert np.allclose(
        DualVlmSorter._camera_base_to_arm(sorter, "right", "right"),
        np.eye(4),
    )


def test_place_planning_start_matches_home_pose_in_twin_radians():
    home_pose_deg = {"J1": 90.0, "J2": -45.0, "J3": 0.0}
    assert np.allclose(
        DualVlmSorter._pose_to_radians(home_pose_deg),
        [np.pi / 2.0, -np.pi / 4.0, 0.0],
    )


def test_abort_cleanup_opens_both_grippers():
    events = []

    class FakeHand:
        def __init__(self, side):
            self.side = side

        def open(self):
            events.append(("open", self.side))

        def is_fully_open(self):
            return True

    class FakeHelper:
        def __init__(self, side):
            self.hand = FakeHand(side)

        def gripper_for(self, side):
            return self.hand

    class FakeSorter:
        pass

    DualVlmSorter._open_grippers_on_abort(
        FakeSorter(),
        {"left": FakeHelper("left"), "right": FakeHelper("right")},
    )
    assert events == [("open", "left"), ("open", "right")]


def test_helper_cleanup_closes_all_camera_instances_without_closing_gripper():
    events = []

    class FakeClient:
        def __init__(self, name):
            self.name = name

        def close(self):
            events.append(("close", self.name))

    class FakeGripper:
        def close(self):
            events.append(("physical_close", "gripper"))

        def close_connection(self):
            events.append(("close_connection", "gripper"))

    class FakeHelper:
        _arms = {"left": FakeClient("arm")}
        _hands = {"left": FakeGripper()}
        _twins = {"left": FakeClient("twin")}
        _perception = None
        _camera = None
        _cameras = {
            "left": FakeClient("camera_left"),
            "right": FakeClient("camera_right"),
        }

    DualVlmSorter._close_helper_clients(FakeHelper())

    assert ("close", "camera_left") in events
    assert ("close", "camera_right") in events
    assert ("close_connection", "gripper") in events
    assert ("physical_close", "gripper") not in events


def test_skill_legacy_camera_alias_reuses_left_camera_cache():
    created = []

    class TestSkill(Skill):
        def run(self, **kwargs):
            return None

    skill = TestSkill.__new__(TestSkill)
    skill._camera = None
    skill._cameras = {}

    def make_camera(side):
        camera = object()
        created.append((side, camera))
        return camera

    skill._make_camera = make_camera
    camera_from_cache = skill.get_camera("left")
    camera_from_legacy_property = skill.camera

    assert camera_from_legacy_property is camera_from_cache
    assert created == [("left", camera_from_cache)]


def test_same_camera_geometry_fallback_merges_semantically_drifted_duplicate():
    sorter = DualVlmSorter.__new__(DualVlmSorter)
    safety = {
        "scene_inventory_sides": ["left"],
        "dedup_distance_m": 0.03,
        "multi_view_merge_distance_m": 0.065,
        "same_camera_label_merge_distance_m": 0.08,
        "cross_view_merge_distance_m": 0.12,
        "cross_view_label_merge_distance_m": 0.35,
        "max_objects": 10,
        "require_visual_destinations": False,
    }
    sorter.task = {
        "scene_inventory_sides": ["left"],
        "groups": {},
        "safety": safety,
    }
    def item(view_id, point):
        return {
            "id": f"{view_id}_lemon",
            "label": "柠檬" if view_id == "left_view_2" else "香蕉",
            "group": "fruit",
            "confidence": 0.92 if view_id == "left_view_2" else 0.85,
            "point_left": list(point),
            "side": "left",
            "observation_id": view_id,
        }
    sorter.observations = {
        "left_view_1": Observation(
            "left", np.zeros((2, 2, 3), dtype=np.uint8),
            np.ones((2, 2), dtype=np.uint16), np.eye(4),
            objects=[item("left_view_1", [0.0, 0.0, 0.1])],
        ),
        "left_view_2": Observation(
            "left", np.zeros((2, 2, 3), dtype=np.uint8),
            np.ones((2, 2), dtype=np.uint16), np.eye(4),
            objects=[item("left_view_2", [0.07, 0.0, 0.1])],
        ),
    }
    sorter._save_scene_inventory = lambda: None

    sorter._merge_scene()

    assert len(sorter.objects) == 1
    assert len(sorter.objects[0].views) == 2
    assert sorter.objects[0].views[1]["merge_method"] == (
        "same_camera_geometry_fallback"
    )
    assert sorter.objects[0].label == "柠檬"
