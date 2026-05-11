"""
Camera capture wrappers.

Usage:
    from core.camera import RealSenseCapture, TianyiCamera, create_camera
    cam = create_camera(config, save_path="./log")
    rgb, depth = cam.get_rgbd()
"""

import os
import json

import cv2
import numpy as np
from PIL import Image

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

# WebSocket imports (only needed for TianyiCamera)
try:
    import asyncio
    import websockets
except ImportError:
    asyncio = None
    websockets = None


class RealSenseCapture:
    """Capture a single aligned RGB-D frame from an Intel RealSense camera."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        enable_align: bool = True,
        save_path: str = "",
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_align = enable_align
        self.save_path = save_path
        if self.save_path and not os.path.exists(self.save_path):
            os.makedirs(self.save_path, exist_ok=True)

    def get_rgbd(self, idx: int = 0):
        """
        Capture a single aligned RGB-D pair.

        Returns
        -------
        rgb : np.ndarray, shape (H, W, 3), dtype uint8
        depth : np.ndarray, shape (H, W), dtype uint16
        """
        pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        profile = pipeline.start(config)
        color_sensor = profile.get_device().query_sensors()[1]
        color_sensor.set_option(rs.option.enable_auto_white_balance, False)
        color_sensor.set_option(rs.option.white_balance, 5500)

        align = rs.align(rs.stream.color) if self.enable_align else None

        # Discard the first few frames to let auto-exposure settle
        for _ in range(5):
            pipeline.wait_for_frames()

        frames = pipeline.wait_for_frames()

        if self.enable_align:
            frames = align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Convert BGR -> RGB
        rgb = np.asanyarray(color_frame.get_data())[..., ::-1]
        depth = np.asanyarray(depth_frame.get_data())

        pipeline.stop()

        # Save to disk (same behaviour as original)
        if self.save_path:
            Image.fromarray(rgb).save(os.path.join(self.save_path, "rgb.png"))
            Image.fromarray(depth).save(os.path.join(self.save_path, "depth.png"))

        return rgb, depth


# ======================================================================
# Tianyi WebSocket camera
# ======================================================================

class TianyiCamera:
    """Capture RGB-D frames from the Tianyi Orin camera via WebSocket.

    Protocol: connects to *ws_url*, receives a JSON header
    ``{"type":"frame_data"}`` followed by JPEG color + PNG depth binary
    frames.  Decodes to RGB (uint8) + depth (uint16).
    """

    def __init__(self, ws_url: str, timeout: int = 10, save_path: str = ""):
        self.ws_url = ws_url
        self.timeout = timeout
        self.save_path = save_path
        if self.save_path and not os.path.exists(self.save_path):
            os.makedirs(self.save_path, exist_ok=True)

    def get_rgbd(self, idx: int = 0):
        """Capture one RGB-D pair (sync wrapper around async WS fetch)."""
        async def _fetch():
            async with websockets.connect(self.ws_url, open_timeout=self.timeout) as ws:
                while True:
                    meta = json.loads(await asyncio.wait_for(ws.recv(), timeout=self.timeout))
                    if meta.get("type") == "frame_data":
                        color_data = await asyncio.wait_for(ws.recv(), timeout=self.timeout)
                        depth_data = await asyncio.wait_for(ws.recv(), timeout=self.timeout)
                        color_arr = np.frombuffer(color_data, dtype=np.uint8)
                        depth_arr = np.frombuffer(depth_data, dtype=np.uint8)
                        color = cv2.imdecode(color_arr, cv2.IMREAD_COLOR)
                        depth = cv2.imdecode(depth_arr, cv2.IMREAD_UNCHANGED)
                        # BGR -> RGB to match RealSenseCapture contract
                        rgb = color[..., ::-1]
                        return rgb, depth
                    # else: heartbeat or other message, continue polling

        try:
            rgb, depth = asyncio.run(_fetch())
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            rgb, depth = loop.run_until_complete(_fetch())

        if self.save_path:
            Image.fromarray(rgb).save(os.path.join(self.save_path, "rgb.png"))
            Image.fromarray(depth).save(os.path.join(self.save_path, "depth.png"))

        return rgb, depth


# ======================================================================
# Camera factory
# ======================================================================

def create_camera(config, save_path: str = ""):
    """Instantiate the right camera based on the profile ``camera`` block.

    Uses ``profile["camera"]`` to decide which implementation to return.
    Falls back to ``RealSenseCapture`` when no camera config is present.
    """
    camera_cfg = config.profile.get("camera", {})
    cam_type = camera_cfg.get("type", "realsense")

    if cam_type == "websocket":
        ws_url = camera_cfg.get("ws_url", "ws://192.168.3.16:8765")
        timeout = camera_cfg.get("timeout", 10)
        return TianyiCamera(ws_url=ws_url, timeout=timeout, save_path=save_path)
    else:
        return RealSenseCapture(width=640, height=480, fps=30, save_path=save_path)
