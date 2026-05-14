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

from core.config import HOST, HAND_PORT, HAND_CLOSE, HAND_OPEN

SERVICE_SRC = "/left_hand/movement_control"


class HandClient:
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
        cprint(f"Control hand response: {resp}", "red")
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
        """Check whether fingers are NOT at the fully-closed position.

        Does NOT re-close the hand -- reads current motor state and compares
        against the closed preset. A non-zero difference means something is
        between the fingers (i.e. the hand is holding an object).
        """
        resp = self.get_state()
        value = resp.get("value", [])
        if not value or len(value) < 6:
            return False
        diff = np.array(value) - np.array(self._hand_config["close"])
        return bool(abs(diff.sum()) > 20)

    def is_fully_open(self) -> bool:
        """Check whether all fingers are at the open position."""
        resp = self.get_state()
        value = resp.get("value", [])
        if not value or len(value) < 6:
            return False
        diff = np.array(value) - np.array(self._hand_config["open"])
        return bool(abs(diff.sum()) <= 50)

    def get_finger_deviation(self) -> float:
        """Return sum of absolute deviations from the closed position.

        Larger values mean the hand is more open / holding a larger object.
        """
        resp = self.get_state()
        value = resp.get("value", [])
        if not value or len(value) < 6:
            return 0.0
        diff = np.array(value) - np.array(self._hand_config["close"])
        return float(abs(diff.sum()))
