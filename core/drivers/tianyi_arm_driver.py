"""Tianyi humanoid robot right arm driver — ROS2 Topic to /arm/cmd_pos."""

import time
import math

from core.drivers.arm_driver import ArmDriver

# Joint name → Tianyi motor ID (right arm: 21-27)
JOINT_NAME_TO_ID = {
    "shoulder_pitch_r_joint": 21,
    "shoulder_roll_r_joint": 22,
    "shoulder_yaw_r_joint": 23,
    "elbow_pitch_r_joint": 24,
    "elbow_yaw_r_joint": 25,
    "wrist_pitch_r_joint": 26,
    "wrist_roll_r_joint": 27,
}

ID_TO_JOINT_NAME = {str(v): k for k, v in JOINT_NAME_TO_ID.items()}

DEFAULT_JOINT_NAMES = [
    "shoulder_pitch_r_joint", "shoulder_roll_r_joint", "shoulder_yaw_r_joint",
    "elbow_pitch_r_joint", "elbow_yaw_r_joint", "wrist_pitch_r_joint",
    "wrist_roll_r_joint",
]


class TianyiArmDriver(ArmDriver):
    """ROS2 Topic-based driver for Tianyi's right arm (7-DOF).

    Publishes ``bodyctrl_msgs.msg.CmdSetMotorPosition`` to ``/arm/cmd_pos``.
    Joint angles are stored in degrees in the profile/config layer; the driver
    converts to radians when publishing.
    """

    def __init__(self, host=None, port=None, service_name=None, joint_names=None):
        self._joint_names = joint_names or DEFAULT_JOINT_NAMES
        self._node = None
        self._publisher = None
        self._joint_state_sub = None
        self._current_positions = {}

    # ------------------------------------------------------------------
    # ArmDriver ABC
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        import rclpy
        from bodyctrl_msgs.msg import CmdSetMotorPosition
        from sensor_msgs.msg import JointState

        if not rclpy.ok():
            rclpy.init(args=[])

        self._node = rclpy.create_node("tianyi_arm_driver")
        self._publisher = self._node.create_publisher(
            CmdSetMotorPosition, "/arm/cmd_pos", 10
        )

        def _joint_cb(msg: JointState):
            for name, pos in zip(msg.name, msg.position):
                self._current_positions[name] = float(pos)

        self._joint_state_sub = self._node.create_subscription(
            JointState, "/joint_states", _joint_cb, 10
        )

        # Spin once to let discovery happen
        rclpy.spin_once(self._node, timeout_sec=0.1)
        return True

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
            self._publisher = None

    def move_to_named_pose(self, pose_dict: dict, speed: int = 30) -> bool:
        """Move the right arm to a named joint-space pose.

        *pose_dict* keys are joint names (e.g. ``"shoulder_pitch_r_joint"``),
        values are **degrees**.
        """
        msg = self._build_cmd_msg(pose_dict, speed=speed / 100.0)
        self._publisher.publish(msg)
        time.sleep(0.05)
        return True

    def execute_trajectory(self, trajectory, speed: int = 20) -> bool:
        """Publish a joint-space trajectory to the arm, waypoint by waypoint.

        *trajectory* may be:
        - a list of dicts  ``[{joint_name: rad}, ...]``  (from twin service)
        - a list of lists  ``[[rad, ...], ...]``  (legacy positional format)

        Joint values are interpreted as **radians** (twin output convention).
        """
        speed_factor = max(0.1, min(1.0, speed / 30.0))

        for waypoint in trajectory:
            if isinstance(waypoint, dict):
                joint_dict = waypoint
            elif isinstance(waypoint, (list, tuple)):
                joint_dict = dict(zip(self._joint_names, waypoint))
            else:
                continue

            msg = self._build_cmd_msg(joint_dict, speed=speed_factor, from_radians=True)
            self._publisher.publish(msg)
            time.sleep(0.05)

        return True

    # ------------------------------------------------------------------
    # Optional: query current state
    # ------------------------------------------------------------------

    def get_current_joint_positions(self) -> dict:
        """Return the latest known joint positions (radians) from /joint_states."""
        import rclpy
        if self._node is not None:
            rclpy.spin_once(self._node, timeout_sec=0.01)
        return dict(self._current_positions)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_cmd_msg(self, joint_dict: dict, speed: float, from_radians: bool = False):
        """Build a CmdSetMotorPosition from {joint_name: angle_value}.

        If *from_radians* is True the values are already in radians (from twin
        trajectory); otherwise they are degrees (from skill config).
        """
        from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition

        msg = CmdSetMotorPosition()
        for name, angle in joint_dict.items():
            motor_id = JOINT_NAME_TO_ID.get(name)
            if motor_id is None:
                continue

            pos_rad = float(angle) if from_radians else math.radians(float(angle))
            cmd = SetMotorPosition()
            cmd.name = motor_id
            cmd.pos = pos_rad
            cmd.spd = float(speed)
            # Wrist joints get slightly higher speed (Tianyi convention)
            if motor_id in (26, 27):
                cmd.spd = float(speed) * 1.05
            cmd.cur = 10.0
            msg.cmds.append(cmd)

        return msg
