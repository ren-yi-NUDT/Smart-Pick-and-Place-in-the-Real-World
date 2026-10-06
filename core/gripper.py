"""
Gripper controller -- TCP client to the gripper movement service.

Protocol:
  SEND/RECV: 4-byte big-endian length prefix + JSON payload

The server remains able to answer legacy raw-JSON clients during migration.
"""

import socket
import time

import numpy as np
from termcolor import cprint

from core.config import HOST, GRIPPER_PORT
from core.tcp_protocol import recv_json_compat, send_json_frame

SERVICE_SRC = "/right_gripper/movement_control"

GRIPPER_OPEN_CMD = [1000, 1000]
GRIPPER_CLOSE_CMD = [0, 0]

# Thresholds for state predicates (gripper position is 0..1000, 1000=open).
FULLY_OPEN_THRESHOLD = 950
GRASPING_DEVIATION_THRESHOLD = 50


class GripperClient:
    """TCP client for the parallel gripper. Falls back to mock mode if connection fails."""

    def __init__(self, host: str = HOST, port: int = GRIPPER_PORT,
                 src: str = SERVICE_SRC, allow_mock: bool = False):
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
        send_json_frame(self.sock, data)
        resp = recv_json_compat(self.sock)
        cprint(f"Control gripper response: {resp}", "red")
        return resp

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------
    def open(self, force: int = None, speed: int = None) -> dict:
        """张开夹爪, 并**确认它真的张开了**。

        服务端回一句 "moved to pos=0/255" 只说明指令写进寄存器了, 不代表手指动了
        (服务端的 wait_until_idle 见到 gOBJ!=0 就返回, 而夹爪静止时 gOBJ 本来就是 3)。
        2026-10-06 现场: 夹爪停在闭合位就去抓, 空合一把 —— 日志里只有一行
        "soft_close: no object detected", 看着像对不准, 其实是压根没张开。
        所以这里读回实际开度, 没张开就重发; 两次都不行就把 opened=False 挂回去报出来。
        """
        cmd = {"src": self._src, "type": "set", "cmd": list(self._cmds["open"])}
        if force is not None:
            cmd["force"] = force
        if speed is not None:
            cmd["speed"] = speed
        resp = self._send_cmd(cmd)
        if not self._wait_open():
            resp = self._send_cmd(cmd)          # 重发一次: 单发丢掉的指令是有的
            if not self._wait_open():
                cprint(f"[GripperClient] 夹爪没能张开, 别带着合拢的手指去抓: {resp}", "red")
                resp = dict(resp, opened=False)
        return resp

    def _wait_open(self, settle: float = 2.0) -> bool:
        """轮询到开度到位。open 的 client 值是 1000, 实测机械上限只到 988。"""
        deadline = time.time() + settle
        while True:
            value = self.get_state().get("value")
            if isinstance(value, list) and value and value[0] >= FULLY_OPEN_THRESHOLD:
                return True
            if time.time() >= deadline:
                return False
            time.sleep(0.15)

    def close(self, force: int = None, speed: int = None, soft: bool = False) -> dict:
        cmd = {"src": self._src, "type": "set", "cmd": list(self._cmds["close"])}
        if force is not None:
            cmd["force"] = force
        if speed is not None:
            cmd["speed"] = speed
        if soft:
            cmd["soft"] = True
        return self._send_cmd(cmd)

    def hold(self, force: int = 40) -> dict:
        """停在**当前位置**加力夹住, 而不是一路合到机械限位。

        is_grasping() 的 soft close 探到物体后, 是停在物体所在的开度上 (force=0)。
        接着再来一次 close() 会被当成"合到底": 从探到的开度一路推到 pos 255 (机械
        限位)。对橙子这种近球形的果子, 那不是"夹紧", 是往里挤 —— 果子被挤出去,
        现场看到的是"夹爪一端先碰到果子, 然后往里收, 结果没夹起来"。
        这里把当前位置读回来原样发回去, 只把力加上, 开度不动。
        """
        state = self.get_state()
        value = state.get("value", [])
        if not isinstance(value, list) or len(value) < 2:
            return state
        cmd = {"src": self._src, "type": "set", "cmd": list(value), "force": force}
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
