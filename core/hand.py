"""
Hand controller -- TCP client to the hand movement service.

Protocol:
  SEND:   raw JSON (no length prefix)
  RECV:   raw JSON (no length prefix)

Usage:
    from core.hand import HandClient
    hand = HandClient()
    hand.connect()
    hand.open()
    hand.close()
    state = hand.get_state()
    grasping = hand.is_grasping()
"""

import json
import socket
import time

import numpy as np
from termcolor import cprint

from core.abc import BaseHand
from core.config import HOST, HAND_PORT, HAND_CLOSE, HAND_OPEN

SERVICE_SRC = "/left_hand/movement_control"


class HandClient(BaseHand):
    """TCP client for the dexterous hand."""

    def __init__(self, host: str = HOST, port: int = HAND_PORT):
        self.host = host
        self.port = port
        self.sock = None   # type: socket.socket | None
        self._hand_config = {
            "close": list(HAND_CLOSE),
            "open": list(HAND_OPEN),
        }

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            cprint(f"[HandClient] Connected to {self.host}:{self.port}", "green")
            return True
        except Exception as e:
            cprint(f"[HandClient] Connection failed: {e}", "red")
            return False

    def close_connection(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    # ------------------------------------------------------------------
    # Low-level send
    # ------------------------------------------------------------------
    def _send_cmd(self, data: dict) -> dict:
        """Send raw JSON, receive raw JSON."""
        msg = json.dumps(data).encode("utf-8")
        self.sock.sendall(msg)
        resp = json.loads(self.sock.recv(1024).decode("utf-8"))
        cprint(f"Control hand response: {resp}", "cyan")
        return resp

    # ------------------------------------------------------------------
    # Hand commands  (matching original planner.py format)
    # ------------------------------------------------------------------
    def open(self) -> dict:
        """Open the hand fully."""
        cmd = {"src": SERVICE_SRC, "type": "set", "cmd": list(self._hand_config["open"])}
        return self._send_cmd(cmd)

    def close(self) -> dict:
        """Close the hand (grasp gesture)."""
        cmd = {"src": SERVICE_SRC, "type": "set", "cmd": list(self._hand_config["close"])}
        return self._send_cmd(cmd)

    def get_state(self) -> dict:
        """Query the current hand motor state."""
        cmd = {"src": SERVICE_SRC, "type": "get"}
        return self._send_cmd(cmd)

    def is_grasping(self) -> bool:
        """Close the hand, then check whether an object was grasped."""
        time.sleep(0.7)
        self.close()
        resp = self.get_state()
        value = resp.get("value", [])
        diff = np.array(value) - np.array(self._hand_config["close"])
        return bool(abs(diff.sum()) > 20)
