"""
Igrape-bot3 arm adapter.

Publishes to ROS2 /arm/cmd_pos with CmdSetMotorPosition messages.
Subscribes to /joint_states for blocking wait (target_arrived).
Joint format conversion happens at the adapter boundary.
"""

import time
import json
import threading

import numpy as np
from termcolor import cprint

from core.abc import BaseArm
from core.backends.igrape._joint_map import skill_to_igrape, skill_traj_to_igrape


class IgrapeArm(BaseArm):
    """Arm adapter using ROS2 /arm/cmd_pos topic."""

    def __init__(self, **kwargs):
        self._cmds = []
        self._connected = False
        self._pub = None
        self._node = None
        self._joint_state = {}
        self._joint_sub = None
        self._CmdMsg = None
        self._MotorCmd = None
        self._joint_cfg = None

    def connect(self) -> bool:
        from core.backends.igrape._ros2_context import ROS2Context
        ctx = ROS2Context.get()
        self._node = ctx.node

        # Import ROS2 message types (requires Igrape workspace on sys.path)
        from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
        self._CmdMsg = CmdSetMotorPosition
        self._MotorCmd = SetMotorPosition

        self._pub = self._node.create_publisher(CmdSetMotorPosition, '/arm/cmd_pos', 10)

        # Subscribe to joint states for blocking wait
        from sensor_msgs.msg import JointState
        self._joint_sub = self._node.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )

        # Load joint config for current mapping
        import os, json as _json
        igrape_root = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.abspath(__file__))))),
            "dependence"
        )
        # Use the igrape_config.json to find igrape root
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.abspath(__file__))))),
            "igrape_config.json"
        )
        with open(config_path) as f:
            cfg = _json.load(f)
        joint_cfg_path = os.path.join(cfg["igrape_root"], "body", "configs", "joint_name.json")
        with open(joint_cfg_path) as f:
            self._joint_cfg = _json.load(f)

        self._connected = True
        cprint("[IgrapeArm] Connected (ROS2 /arm/cmd_pos)", "green")
        return True

    def close(self) -> None:
        self._connected = False

    def _joint_state_cb(self, msg):
        """Update current joint positions from /joint_states."""
        if self._joint_cfg is None:
            return
        for i, name in enumerate(msg.name):
            if name in self._joint_cfg:
                motor_id = str(self._joint_cfg[name]["id"])
                self._joint_state[motor_id] = msg.position[i]

    def _target_arrived(self, target_motor_rad: dict, tol: float = 0.5) -> bool:
        """Check if all target joints have reached their positions."""
        for str_id, target_val in target_motor_rad.items():
            cur = self._joint_state.get(str_id)
            if cur is None or abs(cur - target_val) > tol:
                return False
        return True

    # ------------------------------------------------------------------
    # Command builders (same interface as ArmClient)
    # ------------------------------------------------------------------
    def reset_cmd(self) -> None:
        self._cmds = []

    def start_cmd(self) -> None:
        self._cmds.append({"type": "start", "act": []})

    def add_js_cmd(self, joint_dict: dict, speed: int = 5, block: bool = True) -> None:
        self._cmds.append({
            "type": "js",
            "act": joint_dict,
            "speed": speed,
            "block": block,
        })

    def add_ee_cmd(self, ee_trajectory, speed: int = 5, block: bool = True) -> None:
        self._cmds.append({
            "type": "ee",
            "act": ee_trajectory,
            "speed": speed,
            "block": block,
        })

    def send_cmds(self) -> dict:
        """Flush command queue, converting joints and publishing to ROS2."""
        for cmd in self._cmds:
            if cmd["type"] != "js":
                continue

            motor_dict = skill_to_igrape(cmd["act"])
            ros_speed = min(0.95, max(0.1, cmd["speed"] / 100.0))

            msg = self._CmdMsg()
            for motor_id_str, angle_rad in motor_dict.items():
                mc = self._MotorCmd()
                mc.name = int(motor_id_str)
                mc.pos = float(angle_rad)
                mc.spd = ros_speed
                mc.cur = 10.0
                msg.cmds.append(mc)

            self._pub.publish(msg)

            if cmd.get("block", True):
                self._wait_for_target(motor_dict, timeout=30.0)

        self.reset_cmd()
        return {"value": True, "info": {}}

    def _wait_for_target(self, target_motor_rad: dict, timeout: float = 30.0):
        """Block until joints reach target or timeout."""
        start = time.time()
        while time.time() - start < timeout:
            if self._target_arrived(target_motor_rad):
                return
            time.sleep(0.2)
        cprint(f"[IgrapeArm] target_arrived timeout ({timeout}s)", "yellow")

    def move_to_named_pose(self, pose_dict: dict, speed: int = 30) -> bool:
        try:
            self.reset_cmd()
            self.start_cmd()
            self.add_js_cmd(pose_dict, speed=speed, block=True)
            self.send_cmds()
            return True
        except Exception as e:
            cprint(f"[IgrapeArm] move_to_named_pose failed: {e}", "red")
            return False

    def execute_trajectory(self, trajectory, speed: int = 20) -> bool:
        """Execute a list of joint-space waypoints.

        trajectory: iterable of [J1_deg, ..., J7_deg] or {"J1": deg, ...}
        """
        try:
            igrape_traj = skill_traj_to_igrape(trajectory)
            for motor_dict in igrape_traj:
                msg = self._CmdMsg()
                for motor_id, angle_rad in motor_dict.items():
                    mc = self._MotorCmd()
                    mc.name = int(motor_id)
                    mc.pos = float(angle_rad)
                    mc.spd = min(0.95, max(0.1, speed / 100.0))
                    mc.cur = 10.0
                    msg.cmds.append(mc)
                self._pub.publish(msg)
                self._wait_for_target(motor_dict, timeout=15.0)
            return True
        except Exception as e:
            cprint(f"[IgrapeArm] execute_trajectory failed: {e}", "red")
            return False
