"""
Gripper controller -- TCP client to the gripper movement service.

Protocol:
  SEND:   raw JSON (no length prefix)
  RECV:   raw JSON (no length prefix)
"""

import json
import socket
import time

import numpy as np
from termcolor import cprint

SERVICE_SRC = "/right_gripper/movement_control"

GRIPPER_OPEN_CMD = [1000, 1000]
GRIPPER_CLOSE_CMD = [0, 0]

# Thresholds for state predicates (gripper position is 0..1000, 1000=open).
FULLY_OPEN_THRESHOLD = 950
GRASPING_DEVIATION_THRESHOLD = 50


class GripperClient:
    """TCP client for the parallel gripper. Falls back to mock mode if connection fails."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8001,
                 src: str = SERVICE_SRC, allow_mock: bool = True):
        """
        Args:
            host: TCP host of the gripper server.
            port: TCP port of the gripper server.
            src: ``src`` field sent in protocol JSON. Identifies the gripper
                namespace (e.g. ``/right_gripper/movement_control`` or
                ``/left_gripper/movement_control``). The server currently does
                not route on this field but it is logged for diagnostics.
        """
        self.host = host
        self.port = port
        self._src = src
        self.allow_mock = bool(allow_mock)
        self.sock = None
        self._mock = False
        self._cmds = {
            "open": list(GRIPPER_OPEN_CMD),
            "close": list(GRIPPER_CLOSE_CMD),
        }

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(20.0)
            self.sock.connect((self.host, self.port))
            cprint(f"[GripperClient] Connected to {self.host}:{self.port}", "green")
            return True
        except Exception as e:
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            if not self.allow_mock:
                cprint(f"[GripperClient] Connection failed: {e}", "red")
                return False
            cprint(f"[GripperClient] Connection failed: {e}, using mock mode", "yellow")
            self._mock = True
            return True

    def close_connection(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    # ------------------------------------------------------------------
    # Low-level send
    # ------------------------------------------------------------------
    def _send_cmd(self, data: dict) -> dict:
        if self._mock:
            cprint(f"[GripperClient/Mock] {data}", "yellow")
            return {"value": True, "info": "mock success"}
        msg = json.dumps(data).encode("utf-8")
        self.sock.sendall(msg)
        resp = json.loads(self.sock.recv(1024).decode("utf-8"))
        cprint(f"Control gripper response: {resp}", "red")
        return resp

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------
    def open(self, force: int = None, speed: int = None) -> dict:
        cmd = {"src": self._src, "type": "set", "cmd": list(self._cmds["open"])}
        if force is not None:
            cmd["force"] = force
        if speed is not None:
            cmd["speed"] = speed
        return self._send_cmd(cmd)

    def close(self, force: int = None, speed: int = None, soft: bool = False) -> dict:
        cmd = {"src": self._src, "type": "set", "cmd": list(self._cmds["close"])}
        if force is not None:
            cmd["force"] = force
        if speed is not None:
            cmd["speed"] = speed
        if soft:
            cmd["soft"] = True
        return self._send_cmd(cmd)

    def get_state(self) -> dict:
        cmd = {"src": self._src, "type": "get"}
        return self._send_cmd(cmd)

    def is_grasping(self, force: int = 20) -> bool:
        """Detect whether the gripper is holding an object.

        Uses the gripper's built-in object-detection flag (gOBJ==2) via the
        server's ``soft`` close mode, which closes at low force and reports
        ``object_detected`` in the response. This is more reliable than
        position-threshold heuristics — when the gripper closes empty, it
        drives to its mechanical limit (~pos 230) which would falsely trip
        a position threshold.
        """
        if self._mock:
            return True
        time.sleep(0.3)
        resp = self.close(force=force, soft=True)
        if "object_detected" in resp:
            return bool(resp.get("object_detected"))
        # Fallback for older servers without object_detected: a grasp is
        # indicated only if the gripper stopped SHORT of the mechanical limit.
        # Empty closure reaches ~pos 230/255 (client ≈ 98); a real grasp stops
        # earlier with a larger client value. Use a generous threshold.
        value = resp.get("value", [])
        if isinstance(value, list) and len(value) >= 2:
            return value[0] > 200
        return bool(resp.get("value", False))

    def is_fully_open(self) -> bool:
        """Return True if the gripper is at (or very near) its fully open position.

        Used by destination-action verification (e.g. ``verify_destination_action``)
        as evidence that the object was released.
        """
        if self._mock:
            return True
        resp = self.get_state()
        value = resp.get("value", [])
        if not value or len(value) < 2:
            return False
        return all(v >= FULLY_OPEN_THRESHOLD for v in value[:2])

    def get_finger_deviation(self) -> int:
        """Return how far the gripper is from its fully closed position (0..1000).

        Larger value = more open. Used by grasp-verification logic as a proxy
        for ``finger deviation from closed pose``. 0 = fully closed,
        1000 = fully open.
        """
        if self._mock:
            return 0
        resp = self.get_state()
        value = resp.get("value", [])
        if not value or len(value) < 2:
            return 0
        return int(abs(np.array(value[:2]) - np.array(self._cmds["close"])).sum())
