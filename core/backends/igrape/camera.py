"""
Igrape-bot3 camera adapter.

Captures RGB-D frames via WebSocket (ws://192.168.3.16:8765).
Returns the same (rgb_uint8, depth_uint16) format as RealSenseCapture.
"""

import asyncio
import json
import os

import cv2
import numpy as np

from core.abc import BaseCamera


def _fetch_one_frame(ws_url: str, timeout: int = 10):
    """Connect to camera WebSocket, capture one frame, disconnect."""
    async def _fetch():
        import websockets
        try:
            async with websockets.connect(ws_url, open_timeout=timeout) as ws:
                while True:
                    msg_json = await asyncio.wait_for(ws.recv(), timeout=timeout)
                    msg = json.loads(msg_json)

                    if msg.get("type") == "frame_data":
                        color_data = await asyncio.wait_for(ws.recv(), timeout=timeout)
                        depth_data = await asyncio.wait_for(ws.recv(), timeout=timeout)

                        color_arr = np.frombuffer(color_data, dtype=np.uint8)
                        color_frame = cv2.imdecode(color_arr, cv2.IMREAD_COLOR)

                        depth_arr = np.frombuffer(depth_data, dtype=np.uint8)
                        depth_frame = cv2.imdecode(depth_arr, cv2.IMREAD_UNCHANGED)

                        return color_frame, depth_frame
        except asyncio.TimeoutError:
            print("[IgrapeCamera] Frame capture timeout")
            return None
        except Exception as e:
            print(f"[IgrapeCamera] Error: {e}")
            return None

    try:
        return asyncio.run(_fetch())
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        return loop.run_until_complete(_fetch())


class IgrapeCamera(BaseCamera):
    """Camera adapter using Igrape WebSocket RGB-D stream."""

    def __init__(self, save_path="./log", **kwargs):
        self.save_path = save_path
        self.ws_url = kwargs.get("ws_url", None)
        if self.ws_url is None:
            # Load from igrape_config.json
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.dirname(os.path.abspath(__file__))))),
                "igrape_config.json"
            )
            with open(config_path, "r") as f:
                cfg = json.load(f)
            self.ws_url = cfg["camera_ws_url"]

    def get_rgbd(self, idx: int = 0):
        """Capture one RGB-D frame.

        Returns: (rgb: np.ndarray HxWx3 uint8, depth: np.ndarray HxW uint16)
        """
        result = _fetch_one_frame(self.ws_url)
        if result is None:
            raise RuntimeError("Failed to capture frame from Igrape camera")

        color_frame, depth_frame = result

        # BGR -> RGB (matching RealSenseCapture behavior)
        rgb = color_frame[..., ::-1]
        depth = depth_frame

        if self.save_path:
            from PIL import Image
            os.makedirs(self.save_path, exist_ok=True)
            Image.fromarray(rgb).save(os.path.join(self.save_path, "rgb.png"))
            Image.fromarray(depth).save(os.path.join(self.save_path, "depth.png"))

        return rgb, depth
