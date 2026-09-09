import inspect
import json

from core.drawer_executor import (
    DRAWER_TRAJECTORY_SPEED,
    RealDrawerTrajectoryExecutor,
    SimDrawerTrajectoryExecutor,
    create_drawer_executor,
)
from core.drawer_pipeline import DrawerPipeline
from skills.pose_execute import PoseExecuteSkill, _resolve_drawer_speed


def test_drawer_speed_has_one_shared_default():
    assert DRAWER_TRAJECTORY_SPEED == 0.5
    assert inspect.signature(DrawerPipeline.open).parameters["speed"].default == 0.5
    assert inspect.signature(DrawerPipeline.close).parameters["speed"].default == 0.5
    assert inspect.signature(PoseExecuteSkill.play_open_drawer).parameters[
        "speed"
    ].default == 0.5


def test_drawer_speed_normalizes_openclaw_generic_pose_speed():
    assert _resolve_drawer_speed() == 0.5
    assert _resolve_drawer_speed(0.5) == 0.5
    assert _resolve_drawer_speed(50) == 0.5
    assert _resolve_drawer_speed(10) == 0.5
    assert _resolve_drawer_speed(30) == 1.5
    assert inspect.signature(PoseExecuteSkill.play_close_drawer).parameters[
        "speed"
    ].default == 0.5


class _Config:
    def __init__(self, sim_mode):
        self.sim_mode = sim_mode
        self.shared = {
            "host": "10.0.0.5",
            "sim_server_port": 9031,
            "drawer": {
                "arm": "right",
                "bridge_host": "10.0.0.5",
                "bridge_port": 9011,
                "gripper_host": "10.0.0.5",
                "gripper_port": 9001,
                "sim_port": 9031,
            },
        }

    def get_arm_config(self, side):
        return {"arm_ip": "10.0.0.18", "hand_port": 9001}


def test_factory_selects_backend_from_config():
    assert isinstance(
        create_drawer_executor(_Config(sim_mode=True)),
        SimDrawerTrajectoryExecutor,
    )
    assert isinstance(
        create_drawer_executor(_Config(sim_mode=False)),
        RealDrawerTrajectoryExecutor,
    )


def test_real_executor_uses_configured_endpoints():
    executor = RealDrawerTrajectoryExecutor(_Config(sim_mode=False))

    assert executor.bridge_host == "10.0.0.5"
    assert executor.bridge_port == 9011
    assert executor.gripper_host == "10.0.0.5"
    assert executor.gripper_port == 9001


def test_real_executor_reuses_shared_clients_and_sends_full_no_event_trajectory(tmp_path):
    trajectory_path = tmp_path / "close_drawer.json"
    waypoints = [
        [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 988, 988],
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 988, 988],
        [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 988, 988],
    ]
    trajectory_path.write_text(
        json.dumps({"recorded_gripper": True, "waypoints": waypoints}),
        encoding="utf-8",
    )

    class FakeArm:
        sock = object()

        def __init__(self):
            self.trajectories = []
            self.closed = False

        def execute_trajectory(self, trajectory, speed):
            self.trajectories.append((trajectory, speed))
            return True

        def close(self):
            self.closed = True

    class FakeGripper:
        sock = object()
        _src = "/right_gripper/movement_control"

        def __init__(self):
            self.commands = []
            self.closed = False

        def _send_cmd(self, command):
            self.commands.append(command)
            return {"value": True}

        def close_connection(self):
            self.closed = True

    arm = FakeArm()
    gripper = FakeGripper()
    executor = RealDrawerTrajectoryExecutor(
        _Config(sim_mode=False),
        trajectory_dir=str(tmp_path),
        arm_client=arm,
        gripper_client=gripper,
    )

    assert executor.play("close_drawer") is True
    assert len(arm.trajectories) == 1
    assert arm.trajectories[0][0] == [wp[1:8] for wp in waypoints]
    assert arm.trajectories[0][1] == 10
    assert gripper.commands[0]["cmd"] == [988, 988]
    assert arm.closed is False
    assert gripper.closed is False


def test_sim_executor_uses_configured_sim_server():
    executor = SimDrawerTrajectoryExecutor(_Config(sim_mode=True))

    assert executor.host == "10.0.0.5"
    assert executor.port == 9031


def test_sim_executor_forwards_speed_and_skip_ranges(tmp_path):
    trajectory_path = tmp_path / "demo.json"
    waypoints = [[float(i)] + [float(i + j) for j in range(1, 8)]
                 for i in range(1, 5)]
    trajectory_path.write_text(json.dumps({"waypoints": waypoints}), encoding="utf-8")

    class FakeArm:
        sock = object()

        def __init__(self):
            self.calls = []

        def execute_trajectory(self, trajectory, speed):
            self.calls.append((trajectory, speed))
            return True

        def close(self):
            pass

    arm = FakeArm()
    executor = SimDrawerTrajectoryExecutor(
        _Config(sim_mode=True), trajectory_dir=str(tmp_path), arm_client=arm
    )

    assert executor.play("demo", speed=0.5, skip_ranges=[(2, 2)]) is True
    assert arm.calls[0][1] == 10
    assert arm.calls[0][0] == [waypoints[0][1:8], waypoints[2][1:8], waypoints[3][1:8]]


def test_real_executor_dwells_after_reaching_gripper_event(tmp_path, monkeypatch):
    trajectory_path = tmp_path / "receive.json"
    waypoints = [
        [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 988, 988],
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 988, 988],
        [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 98, 98],
        [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 98, 98],
    ]
    trajectory_path.write_text(
        json.dumps({"recorded_gripper": True, "waypoints": waypoints}),
        encoding="utf-8",
    )

    events = []

    class FakeArm:
        sock = object()

        def execute_trajectory(self, trajectory, speed):
            events.append(("arm", trajectory, speed))
            return True

        def close(self):
            pass

    class FakeGripper:
        sock = object()
        _src = "/right_gripper/movement_control"

        def _send_cmd(self, command):
            events.append(("gripper", command["cmd"]))
            return {"value": True}

        def close_connection(self):
            pass

    monkeypatch.setattr(
        "core.drawer_executor.time.sleep",
        lambda seconds: events.append(("sleep", seconds)),
    )
    executor = RealDrawerTrajectoryExecutor(
        _Config(sim_mode=False),
        trajectory_dir=str(tmp_path),
        arm_client=FakeArm(),
        gripper_client=FakeGripper(),
    )

    assert executor.play("receive", gripper_event_wait=5.0) is True
    assert [event[0] for event in events] == [
        "gripper", "arm", "sleep", "gripper", "arm"
    ]
    assert events[2] == ("sleep", 5.0)
