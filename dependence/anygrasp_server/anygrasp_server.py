#!/home/zz/anaconda3/envs/anygrasp/bin/python3.9
"""Long-running AnyGrasp socket server.

Loads the AnyGrasp network ONCE at startup, then serves detect_grasps
requests over TCP on 127.0.0.1:8030. Without this, every
``python run_skill.py pick_and_place`` invocation pays a 1.5-4 s
``load_net()`` cost.

Protocol (binary, length-prefix framed):

  REQUEST
    [4 bytes BE]  payload length N
    payload (N bytes):
        JSON header + b"\\n" + depth_raw_bytes + rgb_raw_bytes
      JSON header fields:
        model          str    "rs_right" | "rs_left" | other
        depth_shape    [H, W]
        depth_dtype    "uint16"
        rgb_shape      [H, W, 3]
        rgb_dtype      "uint8"

  RESPONSE
    [4 bytes BE]  payload length M
    payload (M bytes):
        JSON {"poses": [...], "error": null | "msg"}

Usage:
    python3 anygrasp_server.py [--host 127.0.0.1] [--port 8030]
"""

import argparse
import json
import os
import socket
import struct
import sys
import threading
import time
from typing import Optional

import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURR_DIR, "..", ".."))
SDK_DIR = os.path.join(PROJECT_ROOT, "dependence", "anygrasp_sdk", "grasp_detection")
sys.path.insert(0, SDK_DIR)

from termcolor import cprint  # noqa: E402

from gsnet import AnyGrasp  # noqa: E402

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8030
DEFAULT_CHECKPOINT = os.path.join(
    PROJECT_ROOT, "dependence", "anygrasp_sdk", "checkpoint_detection.tar"
)

# Camera intrinsics -- must match dependence/anygrasp_sdk/grasp_detection/anygrasp_get_poses.py
_INTRINSICS = {
    "rs_right": (386.4509582519531, 385.8191223144531, 318.2220153808594, 238.8162841796875),
    "rs_left": (392.26812744140625, 392.26812744140625, 325.4682312011719, 242.28213500976562),
}
_DEFAULT_INTRINSICS = (320.0, 320.0, 319.5, 239.5)

_DTYPE_BY_NAME = {
    "uint8": np.uint8,
    "uint16": np.uint16,
    "float32": np.float32,
}


class AnyGraspServer:
    """TCP server that wraps a single long-lived AnyGrasp instance."""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT,
                 checkpoint_path: str = DEFAULT_CHECKPOINT):
        self.host = host
        self.port = port
        self.checkpoint_path = checkpoint_path
        self.anygrasp: Optional[AnyGrasp] = None
        self._lock = threading.Lock()  # serialise inference calls

    # ------------------------------------------------------------------ #
    # Model lifecycle
    # ------------------------------------------------------------------ #
    def load_model(self):
        """Load AnyGrasp checkpoint once. Blocks until ready (~2-4 s)."""
        cprint(f"[anygrasp_server] Loading checkpoint: {self.checkpoint_path}", "cyan")

        # AnyGrasp takes a config namespace object; use a simple holder so we
        # avoid nested-class closure pitfalls (the previous version referenced
        # `checkpoint_path` which was only available as self.checkpoint_path).
        class Cnfg: pass
        cnfg = Cnfg()
        cnfg.checkpoint_path = self.checkpoint_path
        cnfg.max_gripper_width = 0.1
        cnfg.gripper_height = 0.03
        cnfg.top_down_grasp = True
        cnfg.debug = False

        anygrasp = AnyGrasp(cnfg)
        anygrasp.load_net()
        self.anygrasp = anygrasp
        cprint("[anygrasp_server] Model loaded, ready to serve requests", "green")

    # ------------------------------------------------------------------ #
    # Inference (mirrors anygrasp_get_poses.py minus the visualization)
    # ------------------------------------------------------------------ #
    def _run_inference(self, rgb: np.ndarray, depth: np.ndarray, model: str):
        if self.anygrasp is None:
            raise RuntimeError("AnyGrasp not loaded")

        colors = np.asarray(rgb, dtype=np.float32) / 255.0
        depths = np.asarray(depth)

        fx, fy, cx, cy = _INTRINSICS.get(model, _DEFAULT_INTRINSICS)
        scale = 1000.0

        xmin, xmax = -0.19, 0.15
        ymin, ymax = -0.1, 0.15
        zmin, zmax = 0.0, 1.0
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        xmap, ymap = np.meshgrid(np.arange(depths.shape[1]), np.arange(depths.shape[0]))
        points_z = depths / scale
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z

        mask = (points_z > 0) & (points_z < 1)
        points = np.stack([points_x, points_y, points_z], axis=-1)[mask].astype(np.float32)
        colors = colors[mask].astype(np.float32)

        gg, _ = self.anygrasp.get_grasp(
            points, colors, lims=lims,
            apply_object_mask=True, dense_grasp=False, collision_detection=True,
        )

        if len(gg) == 0:
            return []

        gg = gg.nms().sort_by_score()
        gg_pick = gg[0:50]
        results = []
        for g in gg_pick:
            results.append({
                "trans": g.translation.tolist(),
                "score": float(g.score),
                "rotation_matrix": g.rotation_matrix.tolist(),
            })
        return results

    # ------------------------------------------------------------------ #
    # Socket protocol
    # ------------------------------------------------------------------ #
    def _recv_exactly(self, conn, n):
        """Read exactly n bytes from conn, or raise."""
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(min(65536, n - len(buf)))
            if not chunk:
                raise ConnectionError("client closed connection mid-message")
            buf += chunk
        return buf

    def _recv_request(self, conn):
        """Read one framed request. Returns (header_dict, depth_np, rgb_np)."""
        (length,) = struct.unpack(">I", self._recv_exactly(conn, 4))
        payload = self._recv_exactly(conn, length)

        # split header from binary arrays at first newline
        nl_idx = payload.index(b"\n")
        header = json.loads(payload[:nl_idx].decode("utf-8"))
        body = payload[nl_idx + 1:]

        depth_dtype = _DTYPE_BY_NAME[header["depth_dtype"]]
        rgb_dtype = _DTYPE_BY_NAME[header["rgb_dtype"]]
        depth_bytes = int(np.prod(header["depth_shape"])) * np.dtype(depth_dtype).itemsize
        rgb_bytes = int(np.prod(header["rgb_shape"])) * np.dtype(rgb_dtype).itemsize

        if len(body) != depth_bytes + rgb_bytes:
            raise ValueError(
                f"payload body size mismatch: got {len(body)}, "
                f"expected {depth_bytes} (depth) + {rgb_bytes} (rgb)"
            )

        depth = np.frombuffer(body[:depth_bytes], dtype=depth_dtype).reshape(header["depth_shape"])
        rgb = np.frombuffer(body[depth_bytes:], dtype=rgb_dtype).reshape(header["rgb_shape"])
        return header, depth, rgb

    def _send_response(self, conn, payload_dict):
        payload = json.dumps(payload_dict).encode("utf-8")
        conn.sendall(struct.pack(">I", len(payload)))
        conn.sendall(payload)

    def _handle_client(self, conn):
        with conn:
            try:
                while True:
                    header, depth, rgb = self._recv_request(conn)
                    model = header.get("model", "rs_right")
                    t0 = time.time()
                    with self._lock:
                        poses = self._run_inference(rgb, depth, model)
                    dt = time.time() - t0
                    cprint(
                        f"[anygrasp_server] inference: model={model} "
                        f"poses={len(poses)} dt={dt*1000:.0f}ms",
                        "green",
                    )
                    self._send_response(conn, {"poses": poses, "error": None})
            except (ConnectionError, ConnectionResetError):
                pass
            except Exception as e:
                cprint(f"[anygrasp_server] request failed: {e}", "red")
                try:
                    self._send_response(conn, {"poses": [], "error": str(e)})
                except Exception:
                    pass

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def serve_forever(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(5)
        cprint(f"[anygrasp_server] listening on {self.host}:{self.port}", "green")
        while True:
            try:
                conn, addr = srv.accept()
                cprint(f"[anygrasp_server] client connected from {addr}", "cyan")
                t = threading.Thread(target=self._handle_client, args=(conn,), daemon=True)
                t.start()
            except Exception as e:
                cprint(f"[anygrasp_server] accept error: {e}", "red")
                time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    args = parser.parse_args()

    server = AnyGraspServer(args.host, args.port, args.checkpoint)
    server.load_model()
    server.serve_forever()


if __name__ == "__main__":
    main()
