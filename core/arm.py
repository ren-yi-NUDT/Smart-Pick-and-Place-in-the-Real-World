"""
Arm controller -- TCP client to the arm movement service.

Protocol:
  SEND:   4-byte big-endian length prefix + JSON payload
  RECV:   raw JSON (no length prefix)

Usage:
    from core.arm import ArmClient
    arm = ArmClient()
    arm.connect()
    arm.reset_cmd()
    arm.start_cmd()
    arm.add_js_cmd({"J1": 0, ...}, speed=20)
    arm.send_cmds()
    arm.move_to_named_pose("grasp1", speed=30)
"""

import json
import socket
import struct

from termcolor import cprint

from core.config import HOST, ARM_PORT


def _send_cmd(sock: socket.socket, data: dict) -> dict:
    """Send *data* (dict) with a 4-byte big-endian length prefix and
    receive a raw-JSON response."""
    data_bytes = json.dumps(data).encode("utf-8")
    length_prefix = struct.pack(">I", len(data_bytes))
    sock.sendall(length_prefix)
    sock.sendall(data_bytes)
    resp = json.loads(sock.recv(1024).decode("utf-8"))
    cprint(f"Control arm response: {resp}", "red")
    return resp


class ArmClient:
    """Stateless wrapper around the arm TCP service."""

    SERVICE_NAME = "/right_arm/movement_control"

    def __init__(self, host: str = HOST, port: int = ARM_PORT):
        self.host = host
        self.port = port
        self.sock = None   # type: socket.socket | None
        self._cmds = []     # type: list[dict]

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            cprint(f"[ArmClient] Connected to {self.host}:{self.port}", "green")
            return True
        except Exception as e:
            cprint(f"[ArmClient] Connection failed: {e}", "red")
            return False

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None
            cprint("[ArmClient] Connection closed", "yellow")

    # ------------------------------------------------------------------
    # Command builders  (identical to the original ArmController API)
    # ------------------------------------------------------------------
    def reset_cmd(self) -> None:
        """Clear the pending command queue."""
        self._cmds = []

    def start_cmd(self) -> None:
        self._cmds.append({"type": "start", "act": []})

    def add_js_cmd(self, joint_dict: dict, speed: int = 5, block: bool = True) -> None:
        """Append a joint-space command.

        *joint_dict* should be like ``{"J1": 1.745, "J2": -0.504, ...}``
        """
        self._cmds.append({
            "type": "js",
            "act": joint_dict,
            "speed": speed,
            "block": block,
        })

    def add_ee_cmd(self, ee_trajectory, speed: int = 5, block: bool = True) -> None:
        """Append an end-effector command."""
        self._cmds.append({
            "type": "ee",
            "act": ee_trajectory,
            "speed": speed,
            "block": block,
        })

    def send_cmds(self) -> dict:
        """Flush the command queue, appending an implicit ``end`` marker."""
        if self.sock is None:
            raise RuntimeError("ArmClient is not connected -- call connect() first")

        self._cmds.append({"type": "end", "act": []})
        req = {"srv": self.SERVICE_NAME, "cmd": self._cmds}
        resp = _send_cmd(self.sock, req)
        self.reset_cmd()

        state = resp.get("value")
        _info = resp.get("info", {})
        return resp

    # ------------------------------------------------------------------
    # High-level helpers
    # ------------------------------------------------------------------
    def move_to_named_pose(self, pose_dict: dict, speed: int = 30) -> bool:
        """Move the arm to a single joint-space pose (start → js → end)."""
        try:
            self.reset_cmd()
            self.start_cmd()
            self.add_js_cmd(pose_dict, speed=speed, block=True)
            self.send_cmds()
            return True
        except Exception as e:
            cprint(f"[ArmClient] move_to_named_pose failed: {e}", "red")
            return False

    def execute_trajectory(self, trajectory, speed: int = 20) -> bool:
        """Execute a list of joint-space waypoints.

        *trajectory* is an iterable of ``[J1, J2, J3, J4, J5, J6, J7]``.
        """
        try:
            self.reset_cmd()
            self.start_cmd()
            for wp in trajectory:
                self.add_js_cmd(
                    {
                        "J1": wp[0], "J2": wp[1], "J3": wp[2],
                        "J4": wp[3], "J5": wp[4], "J6": wp[5], "J7": wp[6],
                    },
                    speed=speed,
                    block=True,
                )
            self.send_cmds()
            return True
        except Exception as e:
            cprint(f"[ArmClient] execute_trajectory failed: {e}", "red")
            return False
