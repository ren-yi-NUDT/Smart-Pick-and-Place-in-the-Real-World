#!/usr/bin/env python3
"""
Socket server bridging core/gripper.py (TCP client, port 8001) to the Robotiq 85
hardware via Modbus RTU. Started standalone — no ROS dependency.

Protocol (raw JSON, no length prefix — same as inspire_hand_bringup):
  REQUEST:  {"src": "/right_gripper/movement_control",
             "type": "set" | "get",
             "cmd": [v, v]}      # set only; v in 0..1000
  RESPONSE: {"src": "/right_gripper/movement_control",
             "value": True | [v, v] | False,
             "info": "..."}

Value mapping (matches GRIPPER_OPEN_CMD=[1000,1000], GRIPPER_CLOSE_CMD=[0,0]
in core/gripper.py — 1000=extended/open, 0=flexed/closed, like the dexterous hand):
  client [1000, 1000] -> Robotiq pos 0x00 (fully open)
  client [   0,    0] -> Robotiq pos 0xFF (fully closed)
  client [   v,    v] -> Robotiq pos = round((1000 - v) / 1000 * 255)

  get response: Robotiq pos p -> [round((255 - p) / 255 * 1000)] * 2
"""

import argparse
import json
import socket
import sys
import threading
import time

from robotiq_driver import Robotiq85, describe_status

SERVICE_SRC = "/right_gripper/movement_control"

DEFAULT_PORT = 8001
DEFAULT_SERIAL = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
DEFAULT_SLAVE = 9


# --------------------------------------------------------------------------- #
# Value mapping
# --------------------------------------------------------------------------- #
def client_to_pos(values):
    """Map client [v, v] (0..1000, 1000=open) to Robotiq pos (0..255, 0=open)."""
    v = values[0]
    v = max(0, min(1000, int(v)))
    return round((1000 - v) / 1000 * 255)


def pos_to_client(pos):
    """Map Robotiq pos (0..255, 0=open) to client [v, v] (0..1000, 1000=open)."""
    pos = max(0, min(255, int(pos)))
    v = round((255 - pos) / 255 * 1000)
    return [v, v]


# --------------------------------------------------------------------------- #
# Server
# --------------------------------------------------------------------------- #
class GripperServer:
    def __init__(self, gripper: Robotiq85, host: str = "127.0.0.1", port: int = DEFAULT_PORT):
        self.gripper = gripper
        self.host = host
        self.port = port
        self._lock = threading.Lock()  # serialise hardware access across clients
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # ------------------------------------------------------------------ #
    def serve_forever(self):
        self._sock.bind((self.host, self.port))
        self._sock.listen(5)
        print(f"[server] listening on {self.host}:{self.port}", flush=True)
        while True:
            conn, addr = self._sock.accept()
            print(f"[server] client connected from {addr}", flush=True)
            t = threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True)
            t.start()

    def _handle_client(self, conn, addr):
        with conn:
            buf = ""
            while True:
                try:
                    data = conn.recv(1024).decode("utf-8", errors="replace")
                except (ConnectionError, OSError):
                    break
                if not data:
                    break
                buf += data
                # One JSON object per recv is the typical pattern from core/gripper.py,
                # but be tolerant: try to parse complete objects out of the buffer.
                while True:
                    try:
                        obj, end = json.JSONDecoder().raw_decode(buf)
                    except json.JSONDecodeError:
                        break  # incomplete — wait for more bytes
                    buf = buf[end:].lstrip()
                    response = self._process(obj)
                    try:
                        conn.sendall((json.dumps(response) + "\n").encode("utf-8"))
                    except (ConnectionError, OSError):
                        return
            print(f"[server] client {addr} disconnected", flush=True)

    # ------------------------------------------------------------------ #
    def _process(self, req: dict) -> dict:
        cmd_type = req.get("type")
        try:
            if cmd_type == "set":
                cmd = req.get("cmd") or [1000, 1000]
                if len(cmd) != 2:
                    return self._err(f"expected cmd len 2, got {len(cmd)}")
                pos = client_to_pos(cmd)
                kwargs = {}
                if "speed" in req:
                    s = req["speed"]
                    if not isinstance(s, int) or not (0 <= s <= 255):
                        return self._err(f"speed must be int 0..255, got {s!r}")
                    kwargs["speed"] = s
                if "force" in req:
                    f = req["force"]
                    if not isinstance(f, int) or not (0 <= f <= 255):
                        return self._err(f"force must be int 0..255, got {f!r}")
                    kwargs["force"] = f

                # Soft mode: close until object detected (gOBJ==2), then hold at current pos with force=0.
                # Avoids sustained force on the grasped object.
                if req.get("soft"):
                    soft_force = kwargs.get("force", 20)
                    with self._lock:
                        self.gripper.move_to(pos, force=soft_force)
                        s = self.gripper.wait_until_idle(timeout=5.0)
                        if s.get("gOBJ") == 2:
                            pos_now = s["position"]
                            self.gripper.move_to(pos_now, force=0)
                            return self._ok(
                                True,
                                f"soft_close: object at pos={pos_now}/255 (closed with force={soft_force}, now holding at force=0)",
                            )
                        return self._ok(
                            True,
                            f"soft_close: no object detected (gOBJ={s.get('gOBJ')}), pos={s.get('position')}/255",
                        )

                with self._lock:
                    self.gripper.move_to(pos, **kwargs)
                    self.gripper.wait_until_idle(timeout=5.0)
                info = f"moved to pos={pos}/255"
                if kwargs:
                    info += f" ({', '.join(f'{k}={v}' for k, v in kwargs.items())})"
                return self._ok(True, info)
            if cmd_type == "get":
                with self._lock:
                    s = self.gripper.read_status()
                return {"src": SERVICE_SRC, "value": pos_to_client(s["position"]),
                        "info": describe_status(s)}
            return self._err(f"unknown type {cmd_type!r}")
        except Exception as e:
            return self._err(f"{type(e).__name__}: {e}")

    @staticmethod
    def _ok(value, info):
        return {"src": SERVICE_SRC, "value": value, "info": info}

    @staticmethod
    def _err(info):
        return {"src": SERVICE_SRC, "value": False, "info": info}


# --------------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--serial", default=DEFAULT_SERIAL, help=f"default {DEFAULT_SERIAL}")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--slave", type=int, default=DEFAULT_SLAVE)
    p.add_argument("--no-activate", action="store_true",
                   help="skip activation (only use if gripper is already activated)")
    args = p.parse_args()

    g = Robotiq85(port=args.serial, baudrate=args.baud, slave=args.slave)
    g.connect()
    print(f"[server] connected to gripper on {args.serial} (slave {args.slave})", flush=True)

    if not args.no_activate:
        if g.is_activated():
            print("[server] gripper already activated", flush=True)
        else:
            print("[server] activating ...", flush=True)
            if not g.activate(reset_first=True, timeout=10.0):
                print("[server] FAILED to activate — continuing anyway", flush=True)

    # Make sure we start from a known open pose so is_grasping() can detect closure.
    print("[server] moving to fully open as startup pose ...", flush=True)
    try:
        g.open()
        g.wait_until_idle(timeout=5.0)
    except Exception as e:
        print(f"[server] startup open failed: {e}", flush=True)

    server = GripperServer(g, host=args.host, port=args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[server] shutting down", flush=True)
    finally:
        g.disconnect()


if __name__ == "__main__":
    sys.exit(main())
