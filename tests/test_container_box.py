#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""容器框被画面边缘切掉时必须丢掉, 不能拿它的中心当容器中心。

背景 (2026-10-04 实测): 臂在 grasp3 看蓝盘子, YOLO 给的是 [341, 0, 576, 21] —— 21px
高、贴着上边缘, 盘子 95% 在画面外。取框中心 (458, 10) 算出来的"盘子中心"落在盘子远端
边缘之外约 20cm, 机械臂在那里松手, 火龙果落在盘子旁边的桌面上 (流水线还报成功)。

这个测试钉住那个判据: 谁把 _well_framed 去掉, 它就失败。
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.perception import Perception, _well_framed      # noqa: E402

H, W = 480, 640
SHAPE = (H, W, 3)


def _bare_perception(detections):
    """不加载 YOLO, 只测 detect_container 的框过滤。"""
    p = object.__new__(Perception)
    p.detect_objects = lambda *a, **k: [list(d) for d in detections]
    return p


def test_the_box_that_put_the_fruit_beside_the_plate_is_rejected():
    assert not _well_framed([341, 0, 576, 21], SHAPE)


@pytest.mark.parametrize("box", [
    [277, 139, 498, 380],       # 完整在画面里的蓝盘子
    [5, 5, 200, 200],           # 离边 5px, 留够了边距
])
def test_a_fully_visible_box_passes(box):
    assert _well_framed(box, SHAPE)


@pytest.mark.parametrize("box", [
    [0, 100, 200, 300],         # 贴左
    [440, 100, 640, 300],       # 贴右
    [100, 300, 300, 480],       # 贴下
    [100, 0, 300, 200],         # 贴上
    [300, 200, 320, 220],       # 太小
    [1, 1, 200, 200],           # 只离边 1px —— 跟 0px 分不开, 按切掉算
    # 2026-10-06 实测的四个: 严格 >0 全放过去了, 但都是被切掉的
    [503.01, 315.51, 639.86, 415.74],   # grasp2 桃子, 距右 0.14px
    [0.11, 145.56, 471.38, 323.06],     # grasp2 粉盘子, 距左 0.11px
    [39.31, 381.80, 529.90, 479.95],    # grasp4 桃子, 距下 0.05px
    [173.94, 0.02, 579.80, 378.81],     # grasp4 假 bottle, 距上 0.02px
])
def test_boxes_touching_an_edge_or_too_small_are_rejected(box):
    assert not _well_framed(box, SHAPE)


def test_detect_container_drops_the_cut_off_box():
    img = np.zeros(SHAPE, np.uint8)
    p = _bare_perception([[341, 0, 576, 21, 0.39, 0]])
    assert p.detect_container(img, "plate", conf=0.1) == []
    assert p.detect_container(img, "blue plate", conf=0.1) == []


def test_a_well_framed_blue_plate_is_returned():
    img = np.zeros(SHAPE, np.uint8)
    img[150:350, 300:500] = (40, 90, 200)                    # RGB, 色相 221° = blue
    p = _bare_perception([[300, 150, 500, 350, 0.4, 0]])
    assert len(p.detect_container(img, "blue plate", conf=0.1)) == 1
