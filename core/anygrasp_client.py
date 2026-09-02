"""TCP client for the long-running AnyGrasp server.

Talks to ``dependence/anygrasp_server/anygrasp_server.py`` (default
127.0.0.1:8030). The server holds the AnyGrasp network in memory so
``detect_grasps()`` is dominated by inference, not ``load_net()``.

Protocol (matches server):
  REQUEST:  4-byte BE length + (JSON header + b"\\n" + depth bytes + rgb bytes)
  RESPONSE: 4-byte BE length + JSON {"poses": [...], "error": null}

Usage:
    from core.anygrasp_client import AnyGraspClient
    c = AnyGraspClient()
    c.connect()
    poses = c.detect_grasps(rgb, depth, model="rs_right")
"""

import json
import socket
import struct
from typing import List

import numpy as np
from termcolor import cprint


class AnyGraspClient:
    """Stateless-per-call TCP client. Holds one persistent socket."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8030):
        self.host = host
        self.port = port
        self.sock = None

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(120.0)
            self.sock.connect((self.host, self.port))
            cprint(f"[AnyGraspClient] Connected to {self.host}:{self.port}", "green")
            return True
        except Exception as e:
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            cprint(
                f"[AnyGraspClient] Connection failed: {e}. "
                f"Is the server running? Start it via start.bash "
                f"(or python3 dependence/anygrasp_server/anygrasp_server.py).",
                "red",
            )
            raise ConnectionError(
                f"AnyGrasp server not reachable at {self.host}:{self.port}. "
                f"Run start.bash first, or 'python3 dependence/anygrasp_server/anygrasp_server.py'."
            ) from e

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    # ------------------------------------------------------------------
    # Protocol helpers
    # ------------------------------------------------------------------
    def _recv_exactly(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(min(65536, n - len(buf)))
            if not chunk:
                raise ConnectionError("server closed connection mid-message")
            buf += chunk
        return buf

    # ------------------------------------------------------------------
    # High-level API
    # ------------------------------------------------------------------
    def detect_grasps(self, rgb: np.ndarray, depth: np.ndarray,
                      model: str = "rs_right", intrinsics: dict = None,
                      depth_scale: float = None) -> List[dict]:
        """Run AnyGrasp on the RGB-D pair.

        Returns
        -------
        list[dict]
            Each dict has ``"trans"``, ``"score"`` and
            ``"rotation_matrix"``; current servers also return the optional
            official gripper dimensions ``"width"``, ``"height"`` and
            ``"depth"``.
        """
        if self.sock is None:
            raise RuntimeError("AnyGraspClient is not connected -- call connect() first")

        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        depth = np.ascontiguousarray(depth, dtype=np.uint16)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"rgb must be (H, W, 3), got shape {rgb.shape}")
        if depth.ndim != 2:
            raise ValueError(f"depth must be (H, W), got shape {depth.shape}")
        if rgb.shape[:2] != depth.shape:
            raise ValueError(
                f"rgb and depth spatial dims must match: rgb={rgb.shape}, depth={depth.shape}"
            )

        header = {
            "model": model,
            "depth_shape": list(depth.shape),
            "depth_dtype": "uint16",
            "rgb_shape": list(rgb.shape),
            "rgb_dtype": "uint8",
        }
        if isinstance(intrinsics, dict):
            required = ("fx", "fy", "cx", "cy")
            if all(key in intrinsics for key in required):
                header["intrinsics"] = {
                    key: float(intrinsics[key]) for key in required
                }
        if depth_scale is not None:
            try:
                depth_scale = float(depth_scale)
                if np.isfinite(depth_scale) and depth_scale > 0.0:
                    header["depth_scale_m"] = depth_scale
            except (TypeError, ValueError):
                pass
        header_bytes = json.dumps(header).encode("utf-8")
        payload = header_bytes + b"\n" + depth.tobytes() + rgb.tobytes()

        self.sock.sendall(struct.pack(">I", len(payload)))
        self.sock.sendall(payload)

        (resp_len,) = struct.unpack(">I", self._recv_exactly(4))
        resp = json.loads(self._recv_exactly(resp_len).decode("utf-8"))
        if resp.get("error"):
            raise RuntimeError(f"AnyGrasp server error: {resp['error']}")
        return resp.get("poses", [])
