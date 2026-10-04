"""颜色识别必须是算出来的, 不是 YOLO 猜的。

用实拍帧当基准: 同一张桌上一个粉盘子一个蓝盘子, 3500K 白平衡下能分开,
5500K (旧设置) 下分不开 —— 后者正是"颜色绑定随机"的根因。
"""
import sys
from pathlib import Path

import cv2
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
    """旧白平衡下蓝盘子的蓝被抹掉 (饱和度只剩 0.17), 认不出颜色。

    这就是当初颜色绑定乱掉的原因: 颜色分不开, YOLO 的颜色词只能靠语言先验瞎猜。
    现在这种情况会退化成 gray —— 明确说"认不出", 而不是自信地报一个错颜色。
    如果哪天有人把 CAMERA_WHITE_BALANCE 改回 5500, 这个测试会失败。
    """
    from core.config import CAMERA_WHITE_BALANCE
    assert CAMERA_WHITE_BALANCE < 4500, "白平衡调回 5500K 会抹掉蓝盘子的蓝"
    assert dominant_color_name(_img("plate_blue_5500k.png"), BLUE_BOX) == "gray"


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


def _solid(hue_deg, sat=0.8, val=0.9, size=60):
    """造一块纯色 (OpenCV H 是 0-179, 所以要除以 2)。"""
    hsv = np.zeros((size, size, 3), np.uint8)
    hsv[..., 0], hsv[..., 1], hsv[..., 2] = int(hue_deg / 2), int(sat * 255), int(val * 255)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


def test_desaturated_pixels_do_not_drag_the_hue():
    """物体上的高光和阴影是灰的, 色相纯属噪声, 不能算进主色。

    实测一根黄瓜 (真色相 ~115°) 在 grasp2 视角因为低饱和像素占多数, 中位数被拖到
    88°, 直接判成 orange。这里造一块 40% 真色 + 60% 灰的图, 要求仍认出真色。
    """
    for hue, want in [(115, "green"), (44, "yellow"), (14, "orange")]:
        img = _solid(hue)
        img[: int(60 * img.shape[0] / 100)] = 128          # 上半截换成灰
        assert dominant_color_name(img, (0, 0, 60, 60)) == want, hue


def test_orange_yellow_boundary_does_not_split_one_fruit():
    """orange/yellow 的界线必须落在实测橙子 (14°) 和柠檬 (42~47°) 之间。

    定在 45° 时同一颗柠檬 grasp1 量到 42°、grasp2 量到 46°, 两个视角各判一种颜色,
    场景地图里就成了两条记录 —— 用户问"黄柠檬"查不到, 问"橙柠檬"才查得到。
    """
    from core.perception import _HUE_BINS_DEG
    bounds = [hi for hi, name in _HUE_BINS_DEG if name == "orange"]
    assert bounds, "找不到 orange 档"
    assert 14 < bounds[0] <= 42, f"orange 上界 {bounds[0]}° 会切开柠檬的实测区间"


def test_pink_wraps_around_zero():
    """粉在 0/360 两侧, 直接取中位数会翻到 180° (cyan)。"""
    img = np.vstack([_solid(356)[:30], _solid(4)[:30]])   # 上半 356°, 下半 4°
    assert dominant_color_name(img, (0, 0, 60, 60)) == "pink"
