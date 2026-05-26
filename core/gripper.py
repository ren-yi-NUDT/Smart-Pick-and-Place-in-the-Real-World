"""
Gripper controller -- TCP client to the gripper movement service.

Protocol (same as HandClient):
  SEND:   raw JSON (no length prefix)
  RECV:   raw JSON (no length prefix)

Interface is aligned with HandClient so that ArmSide can use either
transparently: open(), close(), is_grasping().
"""

import json
import socket
import time

import numpy as np
from termcolor import cprint

SERVICE_SRC = "/right_gripper/movement_control"

GRIPPER_OPEN_CMD = [1000, 1000]
GRIPPER_CLOSE_CMD = [0, 0]


class GripperClient:
    """TCP client for the parallel gripper. Falls back to mock mode if connection fails."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8001):
        self.host = host
        self.port = port
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
            self.sock.connect((self.host, self.port))
            cprint(f"[GripperClient] Connected to {self.host}:{self.port}", "green")
            return True
        except Exception as e:
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
    # Public interface (aligned with HandClient)
    # ------------------------------------------------------------------
    def open(self) -> dict:
        cmd = {"src": SERVICE_SRC, "type": "set", "cmd": list(self._cmds["open"])}
        return self._send_cmd(cmd)

    def close(self) -> dict:
        cmd = {"src": SERVICE_SRC, "type": "set", "cmd": list(self._cmds["close"])}
        return self._send_cmd(cmd)

    def get_state(self) -> dict:
        cmd = {"src": SERVICE_SRC, "type": "get"}
        return self._send_cmd(cmd)

    def is_grasping(self) -> bool:
        if self._mock:
            return True
        time.sleep(0.5)
        self.close()
        resp = self.get_state()
        value = resp.get("value", [])
        diff = np.array(value) - np.array(self._cmds["close"])
        return bool(abs(diff.sum()) > 50)
