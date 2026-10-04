#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""被物体挡住的深度必须顶成穹顶, 不能留成桌面。

背景 (2026-10-04 实测): D455 最小可测距离 ~286mm, 相机离桌面只有 ~30cm。橙子顶
218mm、桌面 292mm —— 整个果子落在死区, 原始深度上就是一个洞, 框内只有 11.8% 有效、
果子中心 0%。hole_filling_filter 拿周围桌面把它填平, AnyGrasp 看到一张平桌子, 抓取
点落在果子前面的桌面上, 夹爪合在空气里。

这个测试钉住"洞要顶起来": 谁把 dome_fill_box 去掉, 它就失败。
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("pyrealsense2")
from core.camera import dome_fill_box                 # noqa: E402
from core.config import MIN_OCCLUDED_HOLE_PX          # noqa: E402

FX = 386.0
H, W = 480, 640
BOX = (265, 185, 375, 295)          # 110x110, 洞半径 55px, 四周都留了桌面


def _scene(cy=240, cx=320, r=55, box=BOX):
    """倾斜桌面 + 一个圆形洞。洞 = 挡住桌面、但相机测不到的物体。"""
    yy, xx = np.mgrid[0:H, 0:W]
    table = (300 + 0.25 * (H - yy)).astype(np.uint16)     # 下近上远
    hole = (yy - cy) ** 2 + (xx - cx) ** 2 < r ** 2
    raw = table.copy()
    raw[hole] = 0
    return raw, table.copy(), table, hole, box            # raw, 补洞后(桌面), 桌面, 洞, 框


def test_hole_is_lifted_into_a_dome_not_left_flat():
    raw, filled, table, hole, box = _scene()
    out = dome_fill_box(raw, filled, box, FX)

    boundary = int(np.percentile(out[hole], 97))          # 洞的最外圈 ≈ 底面
    apex = int(out[hole].min())
    assert out[240, 320] < boundary, "洞中心该比边缘近 (顶起来), 现在是平的"
    # 顶起来的高度 ≈ 等效半径 * 底面深度 / fx (假设物体大致是球, 高 = 半径)
    expected = 55 * boundary / FX
    assert abs((boundary - apex) - expected) < 10, f"穹顶高 {boundary - apex}mm, 期望 ~{expected:.0f}mm"


def test_dome_rises_smoothly_from_the_boundary():
    raw, filled, _, hole, box = _scene()
    out = dome_fill_box(raw, filled, box, FX)
    yy, xx = np.mgrid[0:H, 0:W]
    d = np.sqrt((yy - 240.0) ** 2 + (xx - 320.0) ** 2)
    assert out[hole & (d < 10)].mean() < out[hole & (d > 50)].mean() - 30
    assert out[hole].min() > 0, "补出来的深度不能是 0 (那还是洞)"


def test_valid_depth_is_untouched():
    raw, filled, table, hole, box = _scene()
    out = dome_fill_box(raw, filled, box, FX)
    assert np.array_equal(out[~hole], table[~hole]), "看得见的像素一律不能动"


def test_a_visible_object_is_left_alone():
    """物体在深度里看得见 (没有洞) -> 一个字都不改。"""
    raw, filled, table, _, box = _scene()
    raw = table.copy()                                    # 没有洞
    out = dome_fill_box(raw, table.copy(), box, FX)
    assert np.array_equal(out, table)


def test_speckle_smaller_than_the_threshold_is_skipped():
    raw, filled, table, hole, box = _scene(r=3)
    assert hole.sum() < MIN_OCCLUDED_HOLE_PX
    out = dome_fill_box(raw, filled, box, FX)
    assert np.array_equal(out, table), "噪点级的小洞不该被当成物体"
