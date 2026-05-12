"""
Digital-twin (trajectory inference) TCP client.

Extracted from ``planner.py`` lines 53-75 (``send_cmd_twin``) and
lines 411-434 (``create_send_config_3``, ``create_twin_service``).

Protocol:
  SEND:   raw JSON (no length prefix)
  RECV:   4-byte big-endian length prefix + JSON payload

Usage:
    from core.twin_client import TwinClient
    twin = TwinClient()
    twin.connect()
    resp = twin.generate_trajectory2(target_pose=[...], current_js=[...], struct="left_arm")
"""

import json
import socket
import struct

from termcolor import cprint

from core.abc import BaseTwinClient
from core.config import HOST, TWIN_PORT


class TwinClient(BaseTwinClient):
    """TCP client for the digital-twin trajectory generation service."""

    def __init__(self, host: str = HOST, port: int = TWIN_PORT):
        self.host = host
        self.port = port
        self.sock = None   # type: socket.socket | None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            cprint(f"[TwinClient] Connected to {self.host}:{self.port}", "green")
            return True
        except Exception as e:
            cprint(f"[TwinClient] Connection failed: {e}", "red")
            return False

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    # ------------------------------------------------------------------
    # Protocol helpers
    # ------------------------------------------------------------------
    def _send_cmd(self, data: dict) -> dict:
        """SEND raw JSON, RECV with 4-byte length prefix + JSON."""
        msg = json.dumps(data).encode("utf-8")
        self.sock.sendall(msg)

        # -- receive length prefix --
        length_bytes = b""
        while len(length_bytes) < 4:
            chunk = self.sock.recv(4 - len(length_bytes))
            if not chunk:
                raise ConnectionError("Connection closed")
            length_bytes += chunk
        data_length = struct.unpack(">I", length_bytes)[0]

        # -- receive payload --
        data_bytes = b""
        while len(data_bytes) < data_length:
            chunk = self.sock.recv(min(4096, data_length - len(data_bytes)))
            if not chunk:
                raise ConnectionError("Connection closed")
            data_bytes += chunk

        resp = json.loads(data_bytes.decode("utf-8"))
        cprint(f"Control twin response: {resp}", "red")
        return resp

    # ------------------------------------------------------------------
    # Configuration builders  (preserved from planner.py)
    # ------------------------------------------------------------------
    @staticmethod
    def build_config_grasp(
        prep_position, prep_orn,
        grasp_position, grasp_orn,
        current_js_rad=None,
        default_traj_js_rad=None,
        struct: str = "left_arm",
    ) -> dict:
        """Build the configuration payload for a *grasp* trajectory.

        Mirrors ``Planner.create_send_config_2``.
        """
        if current_js_rad is None:
            if default_traj_js_rad is None:
                raise ValueError("Either current_js_rad or default_traj_js_rad must be provided")
            current_js_rad = list(default_traj_js_rad)

        return {
            "target_pose": [
                [
                    prep_position[0], prep_position[1], prep_position[2],
                    prep_orn[0], prep_orn[1], prep_orn[2], prep_orn[3],
                ],
                [
                    grasp_position[0], grasp_position[1], grasp_position[2],
                    grasp_orn[0], grasp_orn[1], grasp_orn[2], grasp_orn[3],
                ],
            ],
            "current_js": list(current_js_rad),
            "struct": struct,
        }

    @staticmethod
    def build_config_place(
        place_position, place_orn,
        current_js_rad=None,
        default_traj_js_rad=None,
        struct: str = "left_arm",
    ) -> dict:
        """Build the configuration payload for a *place* trajectory.

        Mirrors ``Planner.create_send_config_3``.
        """
        if current_js_rad is None:
            if default_traj_js_rad is None:
                raise ValueError("Either current_js_rad or default_traj_js_rad must be provided")
            current_js_rad = list(default_traj_js_rad)

        return {
            "target_pose": [
                [
                    place_position[0], place_position[1], place_position[2],
                    place_orn[0], place_orn[1], place_orn[2], place_orn[3],
                ],
            ],
            "current_js": list(current_js_rad),
            "struct": struct,
        }

    # ------------------------------------------------------------------
    # High-level service calls
    # ------------------------------------------------------------------
    def call_service(self, srv_type: str, cnfg: dict) -> dict:
        """Send a generic twin-inference request.

        Mirrors ``Planner.create_twin_service``.
        """
        cmd = {"srv": "twin_inference", "type": srv_type, "cnfg": cnfg}
        return self._send_cmd(cmd)

    def generate_trajectory2(self, cnfg: dict) -> dict:
        """Shortcut for ``trajectory_generation2``."""
        return self.call_service("trajectory_generation2", cnfg)
