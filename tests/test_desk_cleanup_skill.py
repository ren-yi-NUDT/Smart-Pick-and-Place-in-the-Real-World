from core.dual_handover import _scaled_speed
from skills.desk_cleanup import DeskCleanupSkill


class _HandoverStub:
    def __init__(self, calls):
        self.calls = calls

    def run(self, **kwargs):
        self.calls.append(("handover", kwargs))
        return True


class _PlaceStub:
    def __init__(self, calls):
        self.calls = calls

    def place_at_named_pose(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return True


class _DrawerStub:
    def __init__(self, calls):
        self.calls = calls

    def open(self):
        self.calls.append(("drawer", "open"))
        return True

    def close(self):
        self.calls.append(("drawer", "close"))
        return True


def test_fruit_uses_1x_pose_handover_and_right_named_pose(monkeypatch):
    calls = []
    skill = object.__new__(DeskCleanupSkill)
    skill._handover_pipeline_ref = _HandoverStub(calls)

    monkeypatch.setattr(skill, "visual_grasp", lambda *args, **kwargs: True)
    monkeypatch.setattr(skill, "_grip_position", lambda *args, **kwargs: (True, 100))
    monkeypatch.setattr(
        skill,
        "_deliver_right_direct",
        lambda **kwargs: calls.append(("right_named_pose", kwargs)) or True,
    )

    delivered, ok, error = skill._run_fruits([("apple", 0.9)], 1)

    assert (delivered, ok, error) == (1, True, "")
    assert calls == [
        (
            "handover",
            {
                "mode": "dual",
                "speed": 1,
                "require_confirmation": False,
                "direction": "left_to_right",
            },
        ),
        ("right_named_pose", {}),
    ]


def test_dual_handover_speed_is_a_multiplier():
    assert _scaled_speed(15, 1) == 15
    assert _scaled_speed(10, 0.6) == 6


def test_red_pepper_returns_home_before_right_arm_delivery(monkeypatch):
    calls = []
    skill = object.__new__(DeskCleanupSkill)
    skill._drawer_pipeline_ref = _DrawerStub(calls)

    monkeypatch.setattr(
        skill,
        "visual_grasp",
        lambda *args, **kwargs: calls.append(("grasp", kwargs["side"])) or True,
    )
    monkeypatch.setattr(skill, "_grip_position", lambda *args, **kwargs: (True, 100))
    monkeypatch.setattr(
        skill,
        "control_arm",
        lambda **kwargs: calls.append(("arm", kwargs)) or True,
    )
    monkeypatch.setattr(
        skill,
        "_deliver_right_direct",
        lambda: calls.append(("deliver", "right")) or True,
    )

    result = skill._run_drawer([], max_veg=1, take_red_pepper=True)

    assert result == ("delivered", 0, True, "")
    assert calls == [
        ("drawer", "open"),
        ("grasp", "right"),
        ("arm", {"pose_type": "home", "speed": 30, "side": "right"}),
        ("deliver", "right"),
        ("drawer", "close"),
    ]


def test_papers_recheck_until_table_is_clear(monkeypatch):
    skill = object.__new__(DeskCleanupSkill)
    place_calls = []
    skill._place_pipeline_ref = _PlaceStub(place_calls)
    remaining = iter((2, 1, 0))
    grasp_calls = []

    monkeypatch.setattr(skill, "_paper_count_on_desk", lambda: next(remaining))
    monkeypatch.setattr(
        skill,
        "visual_grasp",
        lambda *args, **kwargs: grasp_calls.append((args, kwargs)) or True,
    )
    monkeypatch.setattr(skill, "_grip_position", lambda *args, **kwargs: (True, 180))

    thrown, ok, error = skill._run_papers(1, 5)

    assert (thrown, ok, error) == (2, True, "")
    assert len(grasp_calls) == 2
    assert len(place_calls) == 2
