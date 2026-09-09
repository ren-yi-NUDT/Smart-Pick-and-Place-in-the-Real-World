from skills.base import get_skill
import skills.right_give_to_user as right_give_to_user
from skills.handover import HandoverSkill
from core.handover_pipeline import HandoverPipeline


def test_right_give_to_user_is_registered_with_canonical_pose():
    assert get_skill("right_give_to_user") is right_give_to_user.RightGiveToUserSkill
    skill = object.__new__(right_give_to_user.RightGiveToUserSkill)
    assert skill.DEFAULT_POSE == "handover_pose"
    assert skill.ARM_SIDE == "right"
    assert skill.DEFAULT_SPEED == 15
    assert skill.DEFAULT_RELEASE_WAIT == 2.0


def test_handover_skill_defaults_match_left_arm_semantics():
    calls = []

    class Pipeline:
        def run(self, **kwargs):
            calls.append(kwargs)
            return True

    skill = object.__new__(HandoverSkill)
    skill._handover_pipeline_ref = Pipeline()
    assert skill.execute(side="right") is True
    assert calls[0]["release_wait"] == 2.0
    assert "speed_scale" not in calls[0]


def test_left_release_uses_single_handover_pose(monkeypatch):
    events = []

    class Config:
        def get_pose(self, name, side):
            return {"J1": 0} if name == "handover_pose" else None

    class Context:
        config = Config()

        def control_arm(self, **kwargs):
            events.append(dict(kwargs))
            return True

        def control_hand(self, **kwargs):
            return True

    monkeypatch.setattr("core.handover_pipeline.time.sleep", lambda _: None)

    result = HandoverPipeline(Context()).run(mode="user_release", side="left")

    assert result is True
    poses = [e["pose_type"] for e in events if "pose_type" in e]
    assert poses == ["handover_pose", "home"]


def test_right_release_uses_named_right_arm_pose(monkeypatch):
    events = []

    class Config:
        def get_pose(self, name, side):
            return {"J1": 0} if name == "handover_pose" else None

    class Context:
        config = Config()

        def control_arm(self, **kwargs):
            events.append(("arm", kwargs))
            return True

        def control_hand(self, **kwargs):
            events.append(("hand", kwargs))
            return True

    monkeypatch.setattr("core.handover_pipeline.time.sleep", lambda _: None)

    result = HandoverPipeline(Context()).run(
        mode="user_release",
        side="right",
    )

    assert result is True
    assert events[0][0] == "arm"
    assert events[0][1]["pose_type"] == "handover_pose"
    assert events[0][1]["side"] == "right"
    assert any(item[0] == "hand" and item[1]["cmd_type"] == "open" for item in events)
    assert events[-1][1]["pose_type"] == "home"


def test_right_give_skill_uses_pose_execute_methods(monkeypatch):
    events = []
    skill = object.__new__(right_give_to_user.RightGiveToUserSkill)
    skill.play_pose = lambda *args, **kwargs: events.append(("pose", args, kwargs)) or True
    skill.play_hand_gesture = lambda *args, **kwargs: events.append(("hand", args, kwargs)) or True
    monkeypatch.setattr("skills.right_give_to_user.time.sleep", lambda _: None)

    assert skill.execute() is True
    assert events[0][0] == "pose"
    assert events[0][1][0] == "handover_pose"
    assert events[1][0] == "hand"
    assert events[2][1][0] == "home"


def test_recorded_user_subclass_defaults_are_resolved_dynamically(monkeypatch):
    from skills.receive_user_trajectory import ReceiveUserTrajectorySkill

    calls = []

    class Executor:
        def play(self, name, speed, **kwargs):
            calls.append((name, speed, kwargs))
            return True

    class Context:
        def arm(self, side):
            return object()

        def gripper(self, side):
            return object()

    skill = object.__new__(ReceiveUserTrajectorySkill)
    skill.config = type("Config", (), {"sim_mode": False})()
    skill.context = Context()
    monkeypatch.setattr(
        "skills.recorded_user_trajectory.create_drawer_executor",
        lambda *args, **kwargs: Executor(),
    )

    assert skill.execute() is not None
    assert calls == [("right_receive_user_v3", 0.5, {
        "gripper_enabled": True,
        "gripper_event_wait": 5.0,
    })]
