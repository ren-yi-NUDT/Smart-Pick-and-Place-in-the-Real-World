"""Arm client that routes joint commands to the PyBullet SimServer (port 8031).

Interface mirrors core.arm.ArmClient: degrees everywhere.
"""
import json
import socket
import struct

from termcolor import cprint


class SimArmClient:
    def __init__(self, host="127.0.0.1", port=8031, side="left"):
        self.host = host
        self.port = port
        self.side = side
        self.sock = None
        self._cmds = []

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
        self.sock.sendall(json.dumps(data).encode("utf-8"))
        hdr = b""
        while len(hdr) < 4:
            hdr += self.sock.recv(4 - len(hdr))
        n = struct.unpack(">I", hdr)[0]
        body = b""
        while len(body) < n:
            body += self.sock.recv(min(65536, n - len(body)))
        return json.loads(body.decode("utf-8"))

    # -- ArmClient-compatible surface --------------------------------
    def reset_cmd(self):
        self._cmds = []

    def start_cmd(self):
        self._cmds.append({"type": "start", "act": []})

    def add_js_cmd(self, joint_dict, speed=5, block=True):
        self._cmds.append({"type": "js", "act": joint_dict,
                           "speed": speed, "block": block})

    def add_ee_cmd(self, ee_trajectory, speed=5, block=True):
        self._cmds.append({"type": "ee", "act": ee_trajectory,
                           "speed": speed, "block": block})

    def send_cmds(self):
        # flatten accumulated js commands into a single execute_trajectory
        traj = []
        for c in self._cmds:
            if c["type"] == "js":
                act = c["act"]
                traj.append([act.get(f"J{i}", 0.0) for i in range(1, 8)])
        self.reset_cmd()
        return self._send({"cmd": "execute_trajectory",
                           "side": self.side, "trajectory": traj})

    def move_to_named_pose(self, pose_dict, speed=30):
        self._send({"cmd": "move_to_pose", "side": self.side,
                    "pose": pose_dict, "speed": speed})
        return True

    def execute_trajectory(self, trajectory, speed=20):
        self._send({"cmd": "execute_trajectory", "side": self.side,
                    "trajectory": list(trajectory), "speed": speed})
        return True
