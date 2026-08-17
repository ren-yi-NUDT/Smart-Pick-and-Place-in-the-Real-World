"""Gripper client routing open/close to the PyBullet SimServer."""
import json
import socket
import struct

from termcolor import cprint


class SimGripperClient:
    def __init__(self, host="127.0.0.1", port=8031, src="/left_gripper/movement_control"):
        self.host = host
        self.port = port
        self.src = src
        self.side = "left" if "left" in src else "right"
        self.sock = None
        self._pos = 1000  # internal state: 1000=open

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            return True
        except Exception as e:  # noqa: BLE001
            cprint(f"[SimGripperClient] connect failed: {e}", "red")
            return False

    def close_connection(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("SimServer closed the connection")
            buf += chunk
        return buf

    def _send(self, data):
        self.sock.sendall(json.dumps(data).encode("utf-8"))
        n = struct.unpack(">I", self._recv_exact(4))[0]
        body = self._recv_exact(n)
        return json.loads(body.decode("utf-8"))

    def open(self, force=None, speed=None):
        self._pos = 1000
        return self._send({"cmd": "gripper", "side": self.side,
                           "action": "open", "value": 1000})

    def close(self, force=None, speed=None, soft=False):
        self._pos = 0
        return self._send({"cmd": "gripper", "side": self.side,
                           "action": "close", "value": 0})

    def get_state(self):
        return self._send({"cmd": "get_joint_state", "side": self.side})

    def is_grasping(self):
        # simplified: closed-ish counts as grasping (MVP; real contact detection is phase 2)
        return self._pos < 500

    def is_fully_open(self):
        return self._pos >= 950

    def get_finger_deviation(self):
        return 1000 - self._pos
