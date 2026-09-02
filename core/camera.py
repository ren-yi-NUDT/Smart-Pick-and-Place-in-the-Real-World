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

import numpy as np
import pyrealsense2 as rs
from PIL import Image


class RealSenseCapture:
    """Capture a single aligned RGB-D frame from an Intel RealSense camera.

    The rs.pipeline() is started lazily and kept alive for the
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
        intrinsics: dict = None,
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_align = enable_align
        self.save_path = save_path
        self.serial = serial
        self._configured_intrinsics = dict(intrinsics or {})
        self.intrinsics = dict(self._configured_intrinsics)
        if self.save_path and not os.path.exists(self.save_path):
            os.makedirs(self.save_path, exist_ok=True)

        self._pipeline = None
        self._align = rs.align(rs.stream.color) if self.enable_align else None
        self._started = False
        # RealSense depth values are device units, not guaranteed to be
        # exactly millimetres.  Keep the device-reported metres/unit value so
        # the AnyGrasp server can reconstruct the same metric point cloud.
        self.depth_scale = None

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

        try:
            profile = self._pipeline.start(config)
            # Keep the device-reported values for diagnostics.  The validated
            # single-camera calibration passed by Config remains authoritative
            # for projection; falling back to SDK intrinsics is only useful for
            # callers that did not provide calibration.
            try:
                color_intr = profile.get_stream(
                    rs.stream.color
                ).as_video_stream_profile().get_intrinsics()
                self.device_intrinsics = {
                    "fx": color_intr.fx, "fy": color_intr.fy,
                    "cx": color_intr.ppx, "cy": color_intr.ppy,
                    "width": color_intr.width, "height": color_intr.height,
                }
                if not self._configured_intrinsics:
                    self.intrinsics = dict(self.device_intrinsics)
            except Exception:
                self.device_intrinsics = {}

            try:
                self.depth_scale = float(
                    profile.get_device().first_depth_sensor().get_depth_scale()
                )
            except Exception:
                # Keep the protocol's historical 1 mm/unit fallback when the
                # backend does not expose a depth scale.
                self.depth_scale = None

            color_sensor = profile.get_device().query_sensors()[1]
            color_sensor.set_option(rs.option.enable_auto_white_balance, False)
            color_sensor.set_option(rs.option.white_balance, 5500)

            # Discard the first few frames only once, at startup, to let
            # auto-exposure settle. Subsequent get_rgbd() calls reuse the
            # warmed-up pipeline.
            for _ in range(5):
                self._pipeline.wait_for_frames()
        except Exception:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
            self._started = False
            raise

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
        try:
            frames = self._pipeline.wait_for_frames()

            if self.enable_align:
                frames = self._align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            # Convert BGR -> RGB
            rgb = np.asanyarray(color_frame.get_data())[..., ::-1]
            depth = np.asanyarray(depth_frame.get_data())
        except Exception:
            # A UVC/RealSense stream error can leave the pipeline owning the
            # USB device even though no frame was returned. Release it now so
            # the caller can retry instead of making the next pose fail with
            # "Device or resource busy".
            self.close()
            raise

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
