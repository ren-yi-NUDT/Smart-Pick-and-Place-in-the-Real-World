"""
Igrape-bot3 hand adapter.

Publishes to ROS2 /inspire_hand/ctrl/right_hand with JointState messages.
Maps Skill-DB 0-1000 values to Igrape 0.0-1.0 range.
"""

import time

import numpy as np
from termcolor import cprint

from core.abc import BaseHand
from core.config import HAND_CLOSE, HAND_OPEN


class IgrapeHand(BaseHand):
    """Hand adapter using ROS2 /inspire_hand/ctrl/right_hand topic."""

    def __init__(self, **kwargs):
        self._connected = False
        self._pub = None
        self._node = None
        self._JS = None
        self._joint_state = {}
        self._joint_sub = None

    def connect(self) -> bool:
        from core.backends.igrape._ros2_context import ROS2Context
        ctx = ROS2Context.get()
        self._node = ctx.node

        from sensor_msgs.msg import JointState
        self._JS = JointState
        self._pub = self._node.create_publisher(
            JointState, '/inspire_hand/ctrl/right_hand', 10
        )
        # Subscribe to hand feedback if available
        self._joint_sub = self._node.create_subscription(
            JointState, '/inspire_hand/state/right_hand', self._joint_state_cb, 10
        )

        self._connected = True
        cprint("[IgrapeHand] Connected (ROS2 /inspire_hand/ctrl/right_hand)", "green")
        return True

    def close_connection(self) -> None:
        self._connected = False

    def _joint_state_cb(self, msg):
        """Store hand joint positions keyed by finger name (1-6)."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_state[name] = msg.position[i]

    def _send_hand_values(self, values_0_to_1000: list):
        """Publish hand command: Skill-DB [0-1000] -> Igrape [0.0-1.0]."""
        msg = self._JS()
        msg.name = [str(i + 1) for i in range(len(values_0_to_1000))]
        msg.position = [v / 1000.0 for v in values_0_to_1000]
        self._pub.publish(msg)

    def open(self) -> dict:
        self._send_hand_values(list(HAND_OPEN))
        return {"status": "ok"}

    def close(self) -> dict:
        self._send_hand_values(list(HAND_CLOSE))
        return {"status": "ok"}

    def get_state(self) -> dict:
        """Return current hand joint state from feedback topic.

        If feedback is not available, returns the last commanded position.
        """
        if self._joint_state:
            values = [self._joint_state.get(str(i), 0.0) * 1000.0 for i in range(1, 7)]
            return {"value": values}
        # Fallback: return close position as best guess
        return {"value": list(HAND_CLOSE)}

    def is_grasping(self) -> bool:
        """Close hand, then check if object is grasped.

        Uses joint state feedback if available, otherwise assumes success.
        """
        time.sleep(0.7)
        self.close()
        time.sleep(1.0)

        state = self.get_state()
        value = np.array(state.get("value", [0] * 6))
        close_pos = np.array(list(HAND_CLOSE))
        diff = value - close_pos
        # If we got real feedback, check the diff; otherwise assume grasped
        if self._joint_state:
            return bool(abs(diff.sum()) > 20)
        return True
