"""
RealSense camera capture wrapper.

Migrated from ``camera.py`` verbatim.

Usage:
    from core.camera import RealSenseCapture
    cam = RealSenseCapture(width=640, height=480, fps=30, save_path="./log")
    rgb, depth = cam.get_rgbd()
"""

import os

import cv2
from core.abc import BaseCamera
import numpy as np
import pyrealsense2 as rs
from PIL import Image
from termcolor import cprint

from core.config import MIN_FRAME_BRIGHTNESS


class RealSenseCapture(BaseCamera):
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
        # 曝光模式必须显式复位。手动曝光是存在相机里的, 会跨进程残留 —— 谁 (诊断脚本、
        # 崩掉的进程) 把它留在手动档, 之后每次采集都静默出废图: 档位偏低就是全黑,
        # 偏高就是全白 (实测残留 exposure=10000/gain=128 时平均亮度 252, YOLO 一个框
        # 都出不来, 但日志里看不出任何异常)。
        color_sensor.set_option(rs.option.enable_auto_exposure, 1)

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

        # 欠曝检查。曝光/增益都顶格还暗, 说明房间光不够 —— 这时候 YOLO 的分数会掉到
        # 噪声级 (实测黄瓜 0.012)、AnyGrasp 的深度也是废的, 但下游只会表现成"没检出",
        # 完全看不出是相机的问题。这里主动喊一声, 别让废图静默流下去。
        mean_brightness = float(rgb.mean())
        if mean_brightness < MIN_FRAME_BRIGHTNESS:
            cprint(
                f"[Camera] 采集帧欠曝: 平均亮度 {mean_brightness:.0f}/255 "
                f"(阈值 {MIN_FRAME_BRIGHTNESS}, 曝光/增益已顶格)。"
                f"YOLO 和 AnyGrasp 在这个亮度下不可靠, 请检查补光。",
                "red",
            )

        pipeline.stop()

        # Save to disk (same behaviour as original)
        if self.save_path:
            Image.fromarray(rgb).save(os.path.join(self.save_path, "rgb.png"))
            Image.fromarray(depth).save(os.path.join(self.save_path, "depth.png"))

        return rgb, depth
