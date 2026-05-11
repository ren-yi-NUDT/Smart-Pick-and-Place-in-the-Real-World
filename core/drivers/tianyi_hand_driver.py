"""Tianyi Inspire dexterous hand driver — ROS2 Topic to /inspire_hand/ctrl/right_hand."""

import time

from core.drivers.hand_driver import HandDriver

HAND_JOINT_NAMES = ["1", "2", "3", "4", "5", "6"]

# Finger position values (Tianyi Inspire: 0.0 = closed, 1.0 = open)
DEFAULT_OPEN_POSITIONS = [1.0, 1.0, 1.0, 1.0, 1.0, 0.5]
DEFAULT_CLOSE_POSITIONS = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]


class TianyiHandDriver(HandDriver):
    """ROS2 Topic-based driver for the Tianyi right Inspire hand.

    Publishes ``sensor_msgs.msg.JointState`` to ``/inspire_hand/ctrl/right_hand``.
    """

    def __init__(self, host=None, port=None, service_name=None, gestures=None):
        self._gestures = gestures or {
            "open": DEFAULT_OPEN_POSITIONS,
            "close": DEFAULT_CLOSE_POSITIONS,
        }
        self._node = None
        self._publisher = None
        self._last_positions = list(DEFAULT_OPEN_POSITIONS)

    # ------------------------------------------------------------------
    # HandDriver ABC
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        import rclpy
        from sensor_msgs.msg import JointState

        if not rclpy.ok():
            rclpy.init(args=[])

        self._node = rclpy.create_node("tianyi_hand_driver")
        self._publisher = self._node.create_publisher(
            JointState, "/inspire_hand/ctrl/right_hand", 10
        )

        rclpy.spin_once(self._node, timeout_sec=0.1)
        return True

    def close_connection(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
            self._publisher = None

    def open(self) -> dict:
        self._last_positions = list(self._get_gesture("open"))
        self._publish_positions(self._last_positions)
        return {"value": self._last_positions}

    def close(self) -> dict:
        self._last_positions = list(self._get_gesture("close"))
        self._publish_positions(self._last_positions)
        return {"value": self._last_positions}

    def get_state(self) -> dict:
        return {"value": list(self._last_positions)}

    def is_grasping(self) -> bool:
        # No per-finger feedback on Tianyi hand topic; time wait, then
        # return the open/close difference as a proxy (matches RM-75 logic).
        import time as _time
        _time.sleep(0.7)
        close_cfg = self._get_gesture("close")
        diff = sum(abs(a - b) for a, b in zip(self._last_positions, close_cfg))
        return diff > 0.2

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _get_gesture(self, name: str) -> list:
        gesture = self._gestures.get(name)
        if gesture is None:
            gesture = self._gestures.get("open", DEFAULT_OPEN_POSITIONS)
        return list(gesture)

    def _publish_positions(self, positions: list):
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.name = HAND_JOINT_NAMES[:len(positions)]
        msg.position = [float(p) for p in positions]
        self._publisher.publish(msg)
        time.sleep(0.05)
