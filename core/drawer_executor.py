#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real/sim implementations for recorded drawer trajectory execution.

The task layer should not know about Robotic_Arm SDK, SimServer or TCP
endpoints.  Both implementations expose the same ``play`` API, including
optional gripper-event dwell behavior.
The default speed is the shared ``DRAWER_TRAJECTORY_SPEED`` (0.5x).
"""

from abc import ABC, abstractmethod
import json
import os
import time

from termcolor import cprint

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_TRAJECTORY_DIR = os.path.join(
    PROJECT_ROOT, "recorded_trajectories", "right"
)
# One source of truth for both standalone and composite drawer actions.
DRAWER_TRAJECTORY_SPEED = 0.5


class DrawerTrajectoryExecutor(ABC):
    """Port shared by the real and simulated drawer backends."""

    @abstractmethod
    def play(self, name, speed=DRAWER_TRAJECTORY_SPEED, gripper_enabled=True,
             skip_ranges=None, gripper_event_wait=0.0):
        """Replay a trajectory with optional gripper and waypoint filtering.

        ``gripper_event_wait`` is a dwell inserted after an arm segment reaches
        a recorded gripper event and before that gripper command is sent.
        """
        raise NotImplementedError


class RealDrawerTrajectoryExecutor(DrawerTrajectoryExecutor):
    """Replay drawer waypoints through the local arm bridge service."""

    def __init__(self, config, trajectory_dir=DEFAULT_TRAJECTORY_DIR,
                 arm_client=None, gripper_client=None):
        self.config = config
        self.trajectory_dir = trajectory_dir
        self._arm_client = arm_client
        self._gripper_client = gripper_client
        self.arm_side = config.shared.get("drawer", {}).get("arm", "right")
        arm_cfg = config.get_arm_config(self.arm_side)
        drawer_cfg = config.shared.get("drawer", {})
        self.bridge_host = drawer_cfg.get(
            "bridge_host", config.shared.get("host", "127.0.0.1")
        )
        self.bridge_port = int(drawer_cfg.get(
            "bridge_port", arm_cfg.get("arm_port", 8011)
        ))
        self.gripper_host = drawer_cfg.get(
            "gripper_host", config.shared.get("host", "127.0.0.1")
        )
        self.gripper_port = int(drawer_cfg.get(
            "gripper_port", arm_cfg.get("hand_port", 8001)
        ))
        self.gripper_src = drawer_cfg.get(
            "gripper_src", "/%s_gripper/movement_control" % self.arm_side
        )

    def play(self, name, speed=DRAWER_TRAJECTORY_SPEED, gripper_enabled=True,
             skip_ranges=None, gripper_event_wait=0.0):
        from core.arm import ArmClient
        from core.gripper import GripperClient

        trajectory = _load_trajectory(self.trajectory_dir, name)
        if trajectory is None:
            return False
        waypoints = _filter_waypoints(
            trajectory.get("waypoints", []), skip_ranges
        )
        if len(waypoints) < 2:
            cprint("[drawer/real] trajectory has fewer than two waypoints", "red")
            return False
        if float(speed) <= 0:
            cprint("[drawer/real] speed must be positive", "red")
            return False
        gripper_event_wait = _normalize_gripper_event_wait(
            gripper_event_wait, "real"
        )

        # The right-arm bridge uses this value to pace its CAN-FD stream.
        # Keeping the conversion here makes the skill's multiplier explicit
        # in the wire request: 1.0x -> 20%, 0.5x -> 10%.
        motion_speed = max(1, min(100, int(round(20 * float(speed)))))

        cprint(
            "[drawer/real] %s: %s waypoints x %sx (controller speed %s), bridge %s:%s" % (
                name, len(waypoints), speed,
                motion_speed,
                self.bridge_host, self.bridge_port,
            ),
            "cyan",
        )
        owns_arm = self._arm_client is None
        arm = self._arm_client or ArmClient(
            self.bridge_host, self.bridge_port, side=self.arm_side
        )
        gripper = self._gripper_client
        owns_gripper = gripper is None
        try:
            if (getattr(arm, "sock", None) is None) and not arm.connect():
                return False
            has_gripper = (
                bool(trajectory.get("recorded_gripper", False))
                and gripper_enabled
            )
            if has_gripper:
                if gripper is None:
                    gripper = GripperClient(
                        self.gripper_host, self.gripper_port,
                        src=self.gripper_src, allow_mock=False,
                    )
                if (getattr(gripper, "sock", None) is None) and not gripper.connect():
                    return False

            first_joint = list(waypoints[0][1:8])
            has_gripper_events = has_gripper and any(
                len(waypoint) >= 10 and waypoint[8:10] != waypoints[0][8:10]
                for waypoint in waypoints[1:]
            )

            if has_gripper and len(waypoints[0]) >= 10:
                if not self._send_gripper(gripper, waypoints[0][8:10]):
                    return False
            previous = (
                waypoints[0][8:10]
                if has_gripper and len(waypoints[0]) >= 10 else None
            )

            # Split at recorded gripper events. Each arm segment is sent to
            # the same bridge used by normal pose execution, while the
            # gripper command is issued immediately after its waypoint. If
            # there are no gripper events (e.g. close_drawer), send the full
            # trajectory in one request. A separate blocking single-waypoint
            # move to an already-reached start pose can hang the legacy arm
            # bridge while it waits for an unnecessarily strict final state.
            segment = [first_joint] if has_gripper_events else []
            replay_waypoints = waypoints[1:] if has_gripper_events else waypoints
            for waypoint in replay_waypoints:
                current = (
                    waypoint[8:10]
                    if has_gripper and len(waypoint) >= 10 else None
                )
                segment.append(list(waypoint[1:8]))
                if has_gripper and current != previous:
                    if not arm.execute_trajectory(segment, speed=motion_speed):
                        return False
                    if gripper_event_wait > 0:
                        cprint(
                            "[drawer/real] dwell %.1fs at gripper event"
                            % gripper_event_wait,
                            "cyan",
                        )
                        time.sleep(gripper_event_wait)
                    if not self._send_gripper(gripper, current):
                        return False
                    segment = [list(waypoint[1:8])]
                    previous = current
            if not arm.execute_trajectory(segment, speed=motion_speed):
                return False

            cprint("[drawer/real] trajectory %s completed" % name, "green")
            return True
        except KeyboardInterrupt:
            cprint("[drawer/real] trajectory interrupted", "yellow")
            return False
        finally:
            if owns_arm:
                arm.close()
            if owns_gripper and gripper is not None:
                gripper.close_connection()

    @staticmethod
    def _send_gripper(gripper, values):
        try:
            response = gripper._send_cmd({
                "src": gripper._src,
                "type": "set",
                "cmd": list(values),
            })
            if not response.get("value", False):
                cprint("[drawer/real] gripper rejected command", "yellow")
                return False
            return True
        except Exception as exc:
            cprint("[drawer/real] gripper command failed: %s" % exc, "yellow")
            return False


class SimDrawerTrajectoryExecutor(DrawerTrajectoryExecutor):
    """Replay the same trajectory through the configured PyBullet service."""

    def __init__(self, config, trajectory_dir=DEFAULT_TRAJECTORY_DIR,
                 arm_client=None, gripper_client=None):
        self.config = config
        self.trajectory_dir = trajectory_dir
        self._arm_client = arm_client
        self._gripper_client = gripper_client
        drawer_cfg = config.shared.get("drawer", {})
        self.host = drawer_cfg.get(
            "sim_host", config.shared.get("host", "127.0.0.1")
        )
        self.port = int(drawer_cfg.get(
            "sim_port", config.shared.get("sim_server_port", 8031)
        ))
        self.arm_side = drawer_cfg.get("arm", "right")
        self.gripper_src = drawer_cfg.get(
            "gripper_src", "/%s_gripper/movement_control" % self.arm_side
        )

    def play(self, name, speed=DRAWER_TRAJECTORY_SPEED, gripper_enabled=True,
             skip_ranges=None, gripper_event_wait=0.0):
        from core.sim_arm import SimArmClient
        from core.sim_gripper import SimGripperClient

        trajectory = _load_trajectory(self.trajectory_dir, name)
        if trajectory is None:
            return False
        waypoints = _filter_waypoints(
            trajectory.get("waypoints", []), skip_ranges
        )
        if len(waypoints) < 2:
            cprint("[drawer/sim] trajectory has fewer than two waypoints", "red")
            return False

        owns_arm = self._arm_client is None
        arm = self._arm_client or SimArmClient(
            self.host, self.port, side=self.arm_side
        )
        gripper = self._gripper_client
        owns_gripper = gripper is None
        try:
            if (getattr(arm, "sock", None) is None) and not arm.connect():
                return False
            speed = float(speed)
            if speed <= 0:
                cprint("[drawer/sim] speed must be positive", "red")
                return False
            gripper_event_wait = _normalize_gripper_event_wait(
                gripper_event_wait, "sim"
            )
            motion_speed = max(1, min(100, int(round(20 * speed))))
            has_gripper = (
                bool(trajectory.get("recorded_gripper", False))
                and gripper_enabled
            )
            if has_gripper:
                if gripper is None:
                    gripper = SimGripperClient(
                        self.host, self.port, src=self.gripper_src
                    )
                if (getattr(gripper, "sock", None) is None) and not gripper.connect():
                    return False

            cprint(
                "[drawer/sim] %s: %s waypoints x %sx "
                "(controller speed %s), via %s:%s" % (
                    name, len(waypoints), speed, motion_speed,
                    self.host, self.port
                ),
                "cyan",
            )

            def set_gripper(values):
                if gripper is None:
                    return True
                action = "open" if values[0] > 500 else "close"
                response = gripper._send({
                    "cmd": "gripper",
                    "side": self.arm_side,
                    "action": action,
                    "value": int(values[0]),
                })
                return bool(response.get("value", True))

            if has_gripper:
                if not set_gripper(waypoints[0][8:10]):
                    return False
            segment = []
            previous = (
                tuple(waypoints[0][8:10]) if has_gripper else None
            )
            for waypoint in waypoints:
                current = tuple(waypoint[8:10]) if has_gripper else None
                if has_gripper and current != previous:
                    if segment and not arm.execute_trajectory(
                            segment, speed=motion_speed):
                        return False
                    segment = []
                    if gripper_event_wait > 0:
                        cprint(
                            "[drawer/sim] dwell %.1fs at gripper event"
                            % gripper_event_wait,
                            "cyan",
                        )
                        time.sleep(gripper_event_wait)
                    if not set_gripper(current):
                        return False
                    previous = current
                segment.append(list(waypoint[1:8]))
            if segment and not arm.execute_trajectory(
                    segment, speed=motion_speed):
                return False
            cprint("[drawer/sim] trajectory %s completed" % name, "green")
            return True
        finally:
            if owns_gripper and gripper is not None:
                gripper.close_connection()
            if owns_arm:
                arm.close()


def create_drawer_executor(config, trajectory_dir=DEFAULT_TRAJECTORY_DIR,
                           arm_client=None, gripper_client=None):
    """Build the configured backend; the caller does not branch on sim mode."""
    if config.sim_mode:
        return SimDrawerTrajectoryExecutor(
            config, trajectory_dir, arm_client=arm_client,
            gripper_client=gripper_client,
        )
    return RealDrawerTrajectoryExecutor(
        config, trajectory_dir, arm_client=arm_client,
        gripper_client=gripper_client,
    )


def _load_trajectory(trajectory_dir, name):
    path = os.path.join(trajectory_dir, "%s.json" % name)
    if not os.path.exists(path):
        cprint("[drawer] trajectory not found: %s" % path, "red")
        return None
    try:
        with open(path, "r", encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, ValueError) as exc:
        cprint("[drawer] failed to load trajectory %s: %s" % (path, exc), "red")
        return None


def _normalize_gripper_event_wait(value, backend):
    """Validate an optional dwell without changing legacy defaults."""
    try:
        wait = float(value)
    except (TypeError, ValueError):
        cprint("[drawer/%s] gripper_event_wait must be a number" % backend, "red")
        raise ValueError("gripper_event_wait must be a number")
    if wait < 0:
        cprint("[drawer/%s] gripper_event_wait must be non-negative" % backend, "red")
        raise ValueError("gripper_event_wait must be non-negative")
    return wait


def _filter_waypoints(waypoints, skip_ranges):
    """Apply the recording tool's 1-based, inclusive skip-range contract."""
    if not skip_ranges:
        return waypoints
    normalized = []
    for item in skip_ranges:
        parts = item.split(":", 1) if isinstance(item, str) else list(item)
        if len(parts) != 2:
            raise ValueError("skip range must contain START and END")
        start, end = int(parts[0]), int(parts[1])
        if start < 1 or start > end:
            raise ValueError("skip range must satisfy 1 <= START <= END")
        normalized.append((start, end))
    filtered = [
        waypoint for index, waypoint in enumerate(waypoints, start=1)
        if not any(start <= index <= end for start, end in normalized)
    ]
    cprint(
        "[drawer] 跳过航点: %s；保留 %s/%s 个" % (
            ", ".join("%s-%s" % item for item in normalized),
            len(filtered), len(waypoints),
        ),
        "yellow",
    )
    return filtered
