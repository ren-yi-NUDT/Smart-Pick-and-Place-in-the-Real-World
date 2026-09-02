"""RGB-D camera backed by the PyBullet SimServer's get_rgbd."""
import base64
import io
import json
import os
import socket
import struct
from datetime import datetime

import numpy as np
from PIL import Image
from termcolor import cprint

from core.sim_utils import recv_exact


class SimCamera:
    def __init__(self, width=640, height=480, fps=30, save_path="", serial="",
                 host="127.0.0.1", port=8031, side="left"):
        self.width = width
        self.height = height
        self.save_path = save_path
        self.serial = serial
        self.host = host
        self.port = port
        self.side = side
        self.sock = None
        if self.save_path:
            os.makedirs(self.save_path, exist_ok=True)

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            return True
        except Exception as e:  # noqa: BLE001
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            cprint(f"[SimCamera] connect failed: {e}", "red")
            return False

    def get_rgbd(self):
        self.sock.sendall(json.dumps({"cmd": "get_rgbd", "side": self.side}).encode("utf-8"))
        n = struct.unpack(">I", recv_exact(self.sock, 4))[0]
        body = recv_exact(self.sock, n)
        info = json.loads(body.decode("utf-8"))["info"]
        rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(info["rgb_b64"])))).copy()
        depth = np.frombuffer(base64.b64decode(info["depth_b64"]), dtype=np.uint16)
        depth = depth.reshape(info["height"], info["width"])

        # Keep simulation logging consistent with the real RealSense path:
        # every RGB-D observation gets a raw RGB/depth pair in log/.
        if self.save_path:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            tag = self.serial[-4:] if self.serial else f"sim_{self.side}"
            Image.fromarray(rgb).save(
                os.path.join(self.save_path, f"rgb_{tag}_{ts}.png")
            )
            Image.fromarray(depth).save(
                os.path.join(self.save_path, f"depth_{tag}_{ts}.png")
            )
        return rgb, depth

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
