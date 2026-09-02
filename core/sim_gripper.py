"""Gripper client routing open/close to the PyBullet SimServer."""
import socket

from termcolor import cprint

from core.sim_utils import send_json


class SimGripperClient:
    def __init__(self, host="127.0.0.1", port=8031, src="/left_gripper/movement_control"):
        self.host = host
        self.port = port
        self.src = src
        self.side = "left" if "left" in src else "right"
        self.sock = None
        self._pos = 1000  # internal state: 1000=open
        self._grasping = False

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            return True
        except Exception as e:  # noqa: BLE001
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            cprint(f"[SimGripperClient] connect failed: {e}", "red")
            return False

    def close_connection(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _send(self, data):
        return send_json(self.sock, data)

    def open(self, force=None, speed=None):
        self._pos = 1000
        response = self._send({"cmd": "gripper", "side": self.side,
                               "action": "open", "value": 1000})
        self._grasping = False
        return response

    def close(self, force=None, speed=None, soft=False):
        self._pos = 0
        response = self._send({"cmd": "gripper", "side": self.side,
                               "action": "close", "value": 0})
        self._grasping = bool(response.get("info", {}).get("object_detected", False))
        return response

    def set_suction_target(self, point_left):
        return self._send({"cmd": "set_suction_target", "side": self.side,
                           "point_left": [float(v) for v in point_left]})

    def get_state(self):
        return self._send({"cmd": "get_joint_state", "side": self.side})

    def is_grasping(self, force=None):
        return self._grasping

    def is_fully_open(self):
        return self._pos >= 950

    def get_finger_deviation(self):
        return 1000 - self._pos
