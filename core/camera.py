"""
RealSense camera capture wrapper.

Migrated from ``camera.py`` verbatim.

Usage:
    from core.camera import RealSenseCapture
    cam = RealSenseCapture(width=640, height=480, fps=30, save_path="./log")
    rgb, depth = cam.get_rgbd()
"""

import os
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs
from PIL import Image


class RealSenseCapture:
    """Capture a single aligned RGB-D frame from an Intel RealSense camera.

    The rs.pipeline() is started once in __init__ and kept alive for the
    lifetime of the instance. Restarting the pipeline on every get_rgbd()
    call cost ~0.5-1.5 s per capture (hardware re-init + 5 discarded
    auto-exposure frames); reusing it drops per-call latency to tens of ms.
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        enable_align: bool = True,
        save_path: str = "",
        serial: str = "",
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_align = enable_align
        self.save_path = save_path
        self.serial = serial
        if self.save_path and not os.path.exists(self.save_path):
            os.makedirs(self.save_path, exist_ok=True)

        self._pipeline = None
        self._align = rs.align(rs.stream.color) if self.enable_align else None
        self._started = False

    def _ensure_started(self):
        """Lazily start the pipeline on first get_rgbd() call.

        Done lazily so that simply constructing RealSenseCapture (e.g. in
        unit tests) does not require a physical camera to be present.
        """
        if self._started:
            return
        self._pipeline = rs.pipeline()
        config = rs.config()
        if self.serial:
            config.enable_device(self.serial)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        profile = self._pipeline.start(config)
        color_sensor = profile.get_device().query_sensors()[1]
        color_sensor.set_option(rs.option.enable_auto_white_balance, False)
        color_sensor.set_option(rs.option.white_balance, 5500)

        # Discard the first few frames only once, at startup, to let
        # auto-exposure settle. Subsequent get_rgbd() calls reuse the
        # warmed-up pipeline.
        for _ in range(5):
            self._pipeline.wait_for_frames()

        self._started = True

    def get_rgbd(self):
        """
        Capture a single aligned RGB-D pair.

        Returns
        -------
        rgb : np.ndarray, shape (H, W, 3), dtype uint8
        depth : np.ndarray, shape (H, W), dtype uint16
        """
        self._ensure_started()
        frames = self._pipeline.wait_for_frames()

        if self.enable_align:
            frames = self._align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Convert BGR -> RGB
        rgb = np.asanyarray(color_frame.get_data())[..., ::-1]
        depth = np.asanyarray(depth_frame.get_data())

        # Save to disk with timestamp to avoid overwriting
        if self.save_path:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            tag = self.serial[-4:] if self.serial else "cam"
            rgb_name = f"rgb_{tag}_{ts}.png"
            depth_name = f"depth_{tag}_{ts}.png"
            Image.fromarray(rgb).save(os.path.join(self.save_path, rgb_name))
            Image.fromarray(depth).save(os.path.join(self.save_path, depth_name))

        return rgb, depth

    def close(self):
        """Stop the pipeline and release the camera. Safe to call multiple times."""
        if self._pipeline is not None and self._started:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self._started = False
        self._pipeline = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
