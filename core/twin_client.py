"""
Digital-twin (trajectory inference) TCP client.

Extracted from ``planner.py`` lines 53-75 (``send_cmd_twin``) and
lines 411-434 (``create_send_config_3``, ``create_twin_service``).

Protocol:
  SEND/RECV: 4-byte big-endian length prefix + JSON payload

Usage:
    from core.twin_client import TwinClient
    twin = TwinClient()
    twin.connect()
    resp = twin.generate_trajectory2(target_pose=[...], current_js=[...], struct="left_arm")
"""

import socket

from termcolor import cprint

from core.config import HOST, TWIN_PORT
from core.tcp_protocol import recv_json_compat, send_json_frame


class TwinClient:
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
            # Reachability is a planning request, not an unbounded wait.
            self.sock.settimeout(60.0)
            self.sock.connect((self.host, self.port))
            cprint(f"[TwinClient] Connected to {self.host}:{self.port}", "green")
            return True
        except Exception as e:
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
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
        """Send and receive one canonical JSON frame."""
        send_json_frame(self.sock, data)
        resp = recv_json_compat(self.sock)
        info = resp.get("info", {}) if isinstance(resp, dict) else {}
        trajectory = info.get("trajectory", []) if isinstance(info, dict) else []
        cprint(
            "[TwinClient] trajectory_generation2 "
            f"value={resp.get('value') if isinstance(resp, dict) else '?'} "
            f"waypoints={len(trajectory) if isinstance(trajectory, list) else 0}",
            "cyan" if isinstance(resp, dict) and resp.get("value") else "yellow",
        )
        return resp

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
