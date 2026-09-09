from skills.base import get_skill
import skills.wipe_table as wipe_table


class _Config:
    def __init__(self, sim_mode=False):
        self.sim_mode = sim_mode

    def get_pose(self, name, side="left"):
        assert (name, side) == ("home", "right")
        return {"J1": 0.0, "J2": 0.0, "J3": 0.0,
                "J4": 0.0, "J5": 0.0, "J6": 0.0, "J7": 0.0}


class _Arm:
    def __init__(self, events):
        self.events = events

    def move_to_named_pose(self, pose, speed):
        self.events.append(("home", pose, speed))
        return True


class _Context:
    def __init__(self, config, events):
        self.config = config
        self.arm_client = _Arm(events)

    def arm(self, side):
        assert side == "right"
        return self.arm_client

    def gripper(self, side):
        assert side == "right"
        return "right-gripper"


class _Executor:
    def __init__(self, events, result=True):
        self.events = events
        self.result = result

    def play(self, name, speed, **kwargs):
        self.events.append(("trajectory", name, speed, kwargs))
        return self.result


def _skill(events, sim_mode=False):
    skill = object.__new__(wipe_table.WipeTableSkill)
    skill.config = _Config(sim_mode)
    skill.context = _Context(skill.config, events)
    return skill


def test_wipe_table_is_registered():
    assert get_skill("wipe_table") is wipe_table.WipeTableSkill


def test_wipe_table_wipe_only_returns_home_after_trajectory(monkeypatch):
    events = []
    executor = _Executor(events)
    monkeypatch.setattr(wipe_table, "create_drawer_executor", lambda *a, **k: executor)

    result = _skill(events, sim_mode=True).execute(
        speed=0.5, home_speed=10, grasp_sponge=False,
    )

    assert result.ok is True
    assert result.data["returned_home"] is True
    assert result.data["stages"] == ["wipe"]
    assert events[0][0:3] == ("trajectory", "wipe_table_demo", 0.5)
    assert events[0][3]["skip_ranges"] is None
    assert events[0][3]["gripper_enabled"] is False
    assert events[-1][0] == "home"


def test_wipe_table_does_not_continue_when_trajectory_fails(monkeypatch):
    events = []
    executor = _Executor(events, result=False)
    monkeypatch.setattr(wipe_table, "create_drawer_executor", lambda *a, **k: executor)

    result = _skill(events, sim_mode=True).execute(grasp_sponge=False)

    assert result.ok is False
    assert result.code == "WIPE_TRAJECTORY_FAILED"
    assert len(events) == 1


def test_wipe_table_passes_skip_ranges_to_executor(monkeypatch):
    events = []
    executor = _Executor(events)
    monkeypatch.setattr(wipe_table, "create_drawer_executor", lambda *a, **k: executor)

    result = _skill([], sim_mode=True).execute(
        speed=0.5,
        skip_ranges=[(10, 90), (780, 818)],
        grasp_sponge=False,
    )

    assert result.ok is True
    assert events[0][3]["skip_ranges"] == [(10, 90), (780, 818)]


def test_wipe_table_real_mode_uses_timestamp_replay(monkeypatch):
    events = []
    calls = {}

    def fake_play_real(name, speed, gripper_enabled, skip_ranges):
        calls.update(name=name, speed=speed,
                     gripper_enabled=gripper_enabled,
                     skip_ranges=skip_ranges)
        return True

    monkeypatch.setattr(wipe_table.WipeTableSkill, "_play_real", staticmethod(fake_play_real))

    result = _skill(events).execute(speed=0.5, grasp_sponge=False)

    assert result.ok is True
    assert calls == {"name": "wipe_table_demo", "speed": 0.5,
                     "gripper_enabled": False, "skip_ranges": None}
    assert events[-1][0] == "home"


def test_wipe_table_full_flow_order(monkeypatch):
    events = []
    calls = []

    skill = _skill(events)

    monkeypatch.setattr(skill, "_grasp_sponge", lambda: calls.append("grasp") or None)
    monkeypatch.setattr(
        skill, "_play_trajectory",
        lambda *a, **k: calls.append("wipe") or True,
    )
    monkeypatch.setattr(skill, "_put_back_sponge", lambda: calls.append("put_back") or True)

    result = skill.execute(home_speed=10)

    assert result.ok is True
    assert calls == ["grasp", "wipe", "put_back"]
    assert result.data["stages"] == ["grasp", "wipe", "put_back"]
    assert events[-1][0] == "home"


def test_wipe_table_stops_when_sponge_missing(monkeypatch):
    events = []
    calls = []

    skill = _skill(events)

    monkeypatch.setattr(
        skill, "_grasp_sponge", lambda: "SPONGE_NOT_FOUND"
    )
    monkeypatch.setattr(
        skill, "_play_trajectory",
        lambda *a, **k: calls.append("wipe") or True,
    )

    result = skill.execute()

    assert result.ok is False
    assert result.code == "SPONGE_NOT_FOUND"
    assert calls == []


def test_wipe_table_put_back_defaults_to_grasp_flag(monkeypatch):
    events = []
    calls = []

    skill = _skill(events)

    monkeypatch.setattr(skill, "_grasp_sponge", lambda: calls.append("grasp") or None)
    monkeypatch.setattr(
        skill, "_play_trajectory",
        lambda *a, **k: calls.append("wipe") or True,
    )
    monkeypatch.setattr(
        skill, "_put_back_sponge",
        lambda: calls.append("put_back") or True,
    )

    result = skill.execute(grasp_sponge=False)

    assert result.ok is True
    assert calls == ["wipe"]
