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

from core.config import (
    CAMERA_AUTO_WHITE_BALANCE,
    CAMERA_WHITE_BALANCE,
    HOLE_FILL_MODE,
    MAX_OCCLUDED_HEIGHT_M,
    MIN_FRAME_BRIGHTNESS,
    MIN_OCCLUDED_HOLE_PX,
)
from core.transforms import _get_intrinsics


def dome_fill_box(raw_depth, depth, box, fx=None):
    """把 *box* 里被物体挡住、测不到的深度顶成一个穹顶, 而不是留成桌面。

    为什么需要 (2026-10-04 实测): D455 的最小可测距离 = fx*baseline/max_disparity
    ≈ 386*0.095/128 ≈ 286mm, 而相机架在桌面上方只有 ~30cm。任何有高度的物体, 顶部
    都比桌面更近 —— 实测橙子顶 218mm、桌面 292mm —— 整个果子落在死区里, 原始深度上
    就是一个洞 (框内只有 11.8% 有效, 果子中心 0%)。hole_filling_filter 拿周围的
    **桌面**把它填平, 于是 AnyGrasp 看到的是一张平桌子, 抓取点落在果子**前面**的
    桌面上, 夹爪合在空气里 —— 用户看到的"差了点"。

    按物体底面 (框外一圈最近的桌面深度) 起一个球冠: 顶点比底面高
    (等效半径 * 底面深度 / fx), 即假设物体大致是个球、高度 = 半径。
    实测橙子: 框 113x117px、底面 311mm -> 高度 45mm; AnyGrasp 的抓取点从果子底边
    (v=405~430) 挪到了果子中间 (v=331~374)。

    只改"原始深度就是 0"的像素, 有效深度一律不动 —— 看得见的东西不受影响。
    必须由调用方给出 box: 洞的连通域和背景连成一片 (实测橙子的洞并进了整幅图的边框
    连通域), 靠拓扑分不出哪个洞是物体; 而且盘子内壁也是洞, 顶起来会把放置位置搞错。
    """
    if fx is None:
        fx = _get_intrinsics("right")[0]

    H, W = raw_depth.shape
    x1, y1, x2, y2 = [int(v) for v in box[:4]]
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)
    if x2 - x1 < 4 or y2 - y1 < 4:
        return depth

    hole = raw_depth[y1:y2, x1:x2] == 0
    area = int(hole.sum())
    if area < MIN_OCCLUDED_HOLE_PX:
        return depth                       # 物体在深度里看得见, 不动

    # 框外一圈的有效深度 = 物体底面的桌面。取 10% 分位 = 最近的那圈, 也就是物体真正
    # 坐着的地方 (框的另一侧是桌子更远处, 会把它垫高)。
    pad = 8
    ax1, ay1 = max(0, x1 - pad), max(0, y1 - pad)
    ax2, ay2 = min(W, x2 + pad), min(H, y2 + pad)
    ring = raw_depth[ay1:ay2, ax1:ax2]
    inner = np.zeros(ring.shape, bool)
    inner[y1 - ay1:y2 - ay1, x1 - ax1:x2 - ax1] = True
    vals = ring[~inner]
    vals = vals[vals > 0]
    if len(vals) < 20:
        return depth
    base = float(np.percentile(vals, 10))

    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    rx, ry = (x2 - x1) / 2.0, (y2 - y1) / 2.0
    # 球模型的半径。长条形物体 (黄瓜) 按短边算 -> 矮, 竖着的细高物体 (瓶子) 会算矮,
    # 这是从轮廓猜高度的天花板, 需要时再按类别给真实高度。
    r_px = min(np.sqrt(area / np.pi), rx, ry)
    height = min(r_px * base / fx, MAX_OCCLUDED_HEIGHT_M * 1000.0)

    yy, xx = np.mgrid[y1:y2, x1:x2]
    rho = np.sqrt(((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2)
    sel = hole & (rho < 1.0)               # 只顶框内切椭圆, 四角留给桌面
    if not sel.any():
        return depth

    cap = base - height * np.sqrt(np.clip(1.0 - rho ** 2, 0.0, 1.0))
    blk = depth[y1:y2, x1:x2]
    blk[sel] = np.clip(cap[sel], 1, 65535).astype(np.uint16)
    return depth


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
        self.raw_depth = None      # 上一帧未补洞的深度, 见 dome_fill_box
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
        # 白平衡必须显式复位, 理由同曝光: 它也存在相机里跨进程残留。
        # 值见 core/config.py:CAMERA_WHITE_BALANCE (实测房间灯 ~3500K)。
        color_sensor.set_option(rs.option.enable_auto_white_balance,
                                int(CAMERA_AUTO_WHITE_BALANCE))
        if not CAMERA_AUTO_WHITE_BALANCE:
            color_sensor.set_option(rs.option.white_balance, CAMERA_WHITE_BALANCE)
        # 曝光模式必须显式复位。手动曝光是存在相机里的, 会跨进程残留 —— 谁 (诊断脚本、
        # 崩掉的进程) 把它留在手动档, 之后每次采集都静默出废图: 档位偏低就是全黑,
        # 偏高就是全白 (实测残留 exposure=10000/gain=128 时平均亮度 252, YOLO 一个框
        # 都出不来, 但日志里看不出任何异常)。
        color_sensor.set_option(rs.option.enable_auto_exposure, 1)

        align = rs.align(rs.stream.color) if self.enable_align else None
        hole_filler = rs.hole_filling_filter(HOLE_FILL_MODE)

        # Discard the first few frames to let auto-exposure settle
        for _ in range(5):
            pipeline.wait_for_frames()

        frames = pipeline.wait_for_frames()

        if self.enable_align:
            frames = align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # 补深度空洞。D455 是主动双目, 光滑/深色物体上 IR 匹配不上 —— 实测火龙果
        # 区域只有 14% 的像素有深度, AnyGrasp 拿到的点云就只剩零头。补完 100%,
        # 且深度中位数仍贴着物体自身 (291mm vs 原始 310mm), 没有把物体糊成一块平板。
        # 必须用 mode 2: mode 1 从右侧邻居传播, 会把野值 (实测 22.5m) 铺满整片区域。
        raw_depth = np.asanyarray(depth_frame.get_data()).copy()
        depth_frame = hole_filler.process(depth_frame)

        # Convert BGR -> RGB
        rgb = np.asanyarray(color_frame.get_data())[..., ::-1]
        depth = np.asanyarray(depth_frame.get_data())

        # 补洞填的是**桌面**, 但物体比桌面近 —— 比相机最小可测距离还近的物体整块是洞,
        # 补完就成了"桌子一直延伸到物体前面"。原始深度留一份给 dome_fill_box: 只有它
        # 知道"哪里根本测不到", 补完的那张看不出来。见 core/camera.py:dome_fill_box。
        self.raw_depth = raw_depth

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
