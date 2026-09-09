"""
Arm controller -- TCP client to the arm movement service.

Protocol:
  SEND/RECV: 4-byte big-endian length prefix + JSON payload

The response reader still accepts the historical raw-JSON response so an
already-running bridge can be upgraded independently.

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

import socket

from termcolor import cprint

from core.config import HOST, ARM_PORT
from core.tcp_protocol import JSON_FRAME_PROTOCOL, recv_json_compat, send_json_frame


def _send_cmd(sock: socket.socket, data: dict) -> dict:
    """Send and receive one canonical JSON frame."""
    request = dict(data)
    request["_tcp_protocol"] = JSON_FRAME_PROTOCOL
    send_json_frame(sock, request)
    resp = recv_json_compat(sock)
    cprint(f"Control arm response: {resp}", "red")
    return resp


class ArmClient:
    """Stateless wrapper around the arm TCP service."""

    SERVICE_NAMES = {
        "left": "/left_arm/movement_control",
        "right": "/right_arm/movement_control",
    }
    # Backward-compatible default for callers that do not specify a side.
    SERVICE_NAME = SERVICE_NAMES["right"]

    def __init__(self, host: str = HOST, port: int = ARM_PORT, side: str = "right"):
        if side not in self.SERVICE_NAMES:
            raise ValueError(f"Unsupported arm side: {side}")
        self.host = host
        self.port = port
        self.side = side
        self.service_name = self.SERVICE_NAMES[side]
        self.sock = None   # type: socket.socket | None
        self._cmds = []     # type: list[dict]

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(180.0)
            self.sock.connect((self.host, self.port))
            cprint(f"[ArmClient] Connected to {self.host}:{self.port}", "green")
            return True
        except Exception as e:
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            cprint(f"[ArmClient] Connection failed: {e}", "red")
            return False

    def _set_recv_timeout(self, n_waypoints: int):
        """Set recv timeout: 2 min per waypoint."""
        timeout = max(10, n_waypoints * 120)
        if self.sock is not None:
            self.sock.settimeout(timeout)

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

    def send_cmds(self) -> dict:
        """Flush the command queue, appending an implicit ``end`` marker."""
        if self.sock is None:
            raise RuntimeError("ArmClient is not connected -- call connect() first")

        self._cmds.append({"type": "end", "act": []})
        req = {"srv": self.service_name, "cmd": self._cmds}
        resp = _send_cmd(self.sock, req)
        self.reset_cmd()

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
            response = self.send_cmds()
            return isinstance(response, dict) and response.get("value") is True
        except Exception as e:
            cprint(f"[ArmClient] move_to_named_pose failed: {e}", "red")
            return False

    def execute_trajectory(self, trajectory, speed: int = 20) -> bool:
        """Execute a list of joint-space waypoints.

        *trajectory* is an iterable of ``[J1, J2, J3, J4, J5, J6, J7]``.
        """
        try:
            traj_list = list(trajectory)
            self._set_recv_timeout(len(traj_list))
            self.reset_cmd()
            self.start_cmd()
            for wp in traj_list:
                self.add_js_cmd(
                    {
                        "J1": wp[0], "J2": wp[1], "J3": wp[2],
                        "J4": wp[3], "J5": wp[4], "J6": wp[5], "J7": wp[6],
                    },
                    speed=speed,
                    block=True,
                )
            response = self.send_cmds()
            return isinstance(response, dict) and response.get("value") is True
        except Exception as e:
            cprint(f"[ArmClient] execute_trajectory failed: {e}", "red")
            return False
