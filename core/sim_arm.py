"""Arm client that routes joint commands to the PyBullet SimServer (port 8031).

Interface mirrors core.arm.ArmClient: degrees everywhere.
"""
import socket

from termcolor import cprint

from core.sim_utils import send_json


class SimArmClient:
    def __init__(self, host="127.0.0.1", port=8031, side="left"):
        self.host = host
        self.port = port
        self.side = side
        self.sock = None

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            return True
        except Exception as e:  # noqa: BLE001
            cprint(f"[SimArmClient] connect failed: {e}", "red")
            return False

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _send(self, data):
        return send_json(self.sock, data)

    def move_to_named_pose(self, pose_dict, speed=30):
        self._send({"cmd": "move_to_pose", "side": self.side,
                    "pose": pose_dict, "speed": speed})
        return True

    def execute_trajectory(self, trajectory, speed=20):
        self._send({"cmd": "execute_trajectory", "side": self.side,
                    "trajectory": list(trajectory), "speed": speed})
        return True
