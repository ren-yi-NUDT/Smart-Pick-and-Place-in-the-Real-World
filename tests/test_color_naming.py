"""颜色识别必须是算出来的, 不是 YOLO 猜的。

用实拍帧当基准: 同一张桌上一个粉盘子一个蓝盘子, 3500K 白平衡下能分开,
5500K (旧设置) 下分不开 —— 后者正是"颜色绑定随机"的根因。
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core.perception import dominant_color_name  # noqa: E402

DATA = Path(__file__).resolve().parent / "data"

PINK_BOX = (240, 62, 488, 261)   # plate_pink_3500k.png 里的粉盘子
BLUE_BOX = (275, 137, 497, 377)  # plate_blue_*.png 里的蓝盘子
WOOD_BOX = (30, 300, 200, 420)   # 木桌面, 对照组


def _img(name):
    return np.array(Image.open(DATA / name))


def test_3500k_separates_pink_from_blue():
    assert dominant_color_name(_img("plate_pink_3500k.png"), PINK_BOX) == "pink"
    assert dominant_color_name(_img("plate_blue_3500k.png"), BLUE_BOX) == "blue"


def test_5500k_could_not_tell_them_apart():
    """旧白平衡下两个盘子都是 'pink' —— 这就是当初颜色绑定乱掉的原因。

    如果哪天有人把 CAMERA_WHITE_BALANCE 改回 5500, 这个测试会失败, 提醒他
    颜色识别已经废了。
    """
    from core.config import CAMERA_WHITE_BALANCE
    assert CAMERA_WHITE_BALANCE < 4500, "白平衡调回 5500K 会抹掉蓝盘子的蓝"
    assert dominant_color_name(_img("plate_blue_5500k.png"), BLUE_BOX) == "pink"


def test_color_is_read_from_pixels_not_from_yolo_prompt():
    """同一个框, 换个词问 YOLO 会给出完全不同的分数; 我们的颜色判定只依赖像素。"""
    frame = _img("plate_blue_3500k.png")
    assert dominant_color_name(frame, BLUE_BOX) == "blue"
    # 框平移一点点结果不变 —— 说明它真的在看这一片像素
    shifted = (BLUE_BOX[0] + 5, BLUE_BOX[1] + 5, BLUE_BOX[2] - 5, BLUE_BOX[3] - 5)
    assert dominant_color_name(frame, shifted) == "blue"


def test_wood_table_is_not_confused_with_a_plate():
    """木桌面是橙色系, 不能和粉盘子撞。"""
    assert dominant_color_name(_img("plate_pink_3500k.png"), WOOD_BOX) == "orange"
