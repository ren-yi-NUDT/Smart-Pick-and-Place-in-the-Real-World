"""RGB-D camera backed by the PyBullet SimServer's get_rgbd."""
import base64
import json
import socket
import struct

import numpy as np
from termcolor import cprint


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

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            return True
        except Exception as e:  # noqa: BLE001
            cprint(f"[SimCamera] connect failed: {e}", "red")
            return False

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("SimServer closed the connection")
            buf += chunk
        return buf

    def get_rgbd(self):
        self.sock.sendall(json.dumps({"cmd": "get_rgbd", "side": self.side}).encode("utf-8"))
        n = struct.unpack(">I", self._recv_exact(4))[0]
        body = self._recv_exact(n)
        info = json.loads(body.decode("utf-8"))["info"]
        from PIL import Image
        import io
        rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(info["rgb_b64"])))).copy()
        depth = np.frombuffer(base64.b64decode(info["depth_b64"]), dtype=np.uint16)
        depth = depth.reshape(info["height"], info["width"])
        return rgb, depth

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
