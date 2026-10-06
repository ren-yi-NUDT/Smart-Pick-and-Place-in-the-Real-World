"""场景地图: look_around 扫一遍记下每样东西在世界里的位置, 后面下指令直接查。

为什么需要: 机械臂在 grasp1 只看得见粉盘子, 在 grasp4 只看得见蓝盘子。用户说
"放粉盘子里"时臂不一定正好看着粉盘子 —— 每次靠当场检测就是碰运气。扫一遍记下来,
之后按 (名字, 颜色) 查表就行。

这个模块不碰硬件: 扫描在 skills/look_around.py 里做, 这里只管合并/存取/查找,
所以可以直接用假数据测。
"""
import json
import os
import time

import numpy as np

from core.config import YOLO_CONF_SCAN
from core.perception import _well_framed, dominant_color_name
from core.transforms import pixel_to_camera_point2

SCENE_PATH = "./log/scene.json"

# 扫描词表。一次全给 YOLO —— 实测亮图上一遍就能出结果, 不用逐类跑 (逐类慢 15 倍)。
#
# 这张表同时是**指令词表**: parse_target 拿它当名词表, 地图里的名字也用它。
# 所以每个词必须同时满足两条 —— YOLO-World 认得出, 而且用户下指令时会这么说。
# 2026-10-04 在这张桌子上实测:
#   plum 0.836 / pitaya 0.166 / cucumber 0.906 / orange 0.665 / lemon 0.761
# 换掉的: dragon fruit **零检测** (火龙果要认只能叫 pitaya);
#         peach 只有 0.325, 而且 nectarine 会误匹配到粉盘子, 统一用 plum。
# 注意 apple 在这张桌上很"滥": 桃子 0.968、黄瓜 0.423 都能匹配上。留着是因为真放苹果时
# 要靠它; 但它会让地图里多出同位置的重名条目 —— 只有在用户真的说 "apple" 时才会找错。
#
# 这是**现场标定**的, 和白平衡常数一个性质: 换场景/换东西要重探, 用 tools/probe_vocab.py。
SCAN_VOCAB = [
    "plate", "bowl", "cup", "bottle", "box",
    "plum", "pitaya", "cucumber", "orange", "lemon", "banana", "apple",
]

# 颜色词。把指令里的一串词拆成 (颜色, 名字) 时用。中英都收, 因为用户两种都会说。
COLOR_VOCAB = [
    "blue", "pink", "green", "red", "yellow", "orange", "white",
    "black", "gray", "grey", "purple", "brown",
    "蓝", "蓝色", "粉", "粉色", "绿", "绿色", "红", "红色",
    "黄", "黄色", "橙", "橙色", "白", "白色", "黑", "黑色",
    "紫", "紫色", "灰", "灰色", "棕", "棕色",
]

# 不同视角看到同一个东西, 世界坐标差多少以内算同一个。实测同一个盘子从 grasp1/2
# 看差 3.4cm, 从 grasp3/4 看差 7.6cm (掠射角下深度更飘), 所以 6cm 不够, 用 9cm。
MERGE_TOL_M = 0.09

# 地图里只留工作台范围内的东西。臂的作业半径 ~0.6m, 超过这个数的点一定是假的:
# 实测 grasp3 在画面左边缘检出一个 "bottle", 世界坐标 x=-1.135 (离底座 1.19m),
# 那是背景墙上一道 64x43 像素的假框, 置信度 0.43 还压过了扫描阈值。
MAX_RADIUS_M = 0.9


def _dist(a, b):
    return float(np.linalg.norm(np.asarray(a, float) - np.asarray(b, float)))


def in_workspace(world, max_radius=MAX_RADIUS_M):
    """世界坐标是不是落在臂够得着的范围内。"""
    return _dist(world, [0.0, 0.0, 0.0]) <= max_radius


def merge(entries, tol=MERGE_TOL_M):
    """把多个视角的记录并成一份; **同名**且位置相近的取平均。

    工作台外的记录直接丢掉 —— 那些是背景上的假框, 留着会让人按地图去够一个
    根本不在桌上的坐标。

    判据里**没有颜色**。颜色是每一帧现从框内像素算的, 同一个东西换个视角就会被叫成
    另一个颜色 —— 2026-10-06 的桃子, grasp2 看是 "pink apple" (0.88), grasp3 看是
    "yellow apple" (0.85), 世界坐标差 19mm, 却因为颜色不同各自成了一条。于是地图里
    同一个桃子有两条记录、各自只带一个观察位姿; find 取置信度高的那条 (grasp2), 而
    grasp2 上 AnyGrasp 一个落在桃子上的候选都给不出来, 抓取直接失败。合并本来就是
    merge 的本职, 让颜色把它挡掉是失职。颜色和置信度跟着**看得最准的那一次**走。
    """
    out = []
    for e in entries:
        if not in_workspace(e["world"]):
            continue
        for o in out:
            if o["name"] == e["name"] and _dist(o["world"], e["world"]) < tol:
                n = len(o["seen_from"])
                o["world"] = ((np.array(o["world"]) * n + np.array(e["world"]))
                              / (n + 1)).tolist()
                o["seen_from"].append(e["seen_from"][0])
                if e["conf"] > o["conf"]:
                    o["conf"] = e["conf"]
                    o["color"] = e.get("color")
                break
        else:
            out.append(dict(e))
    return out


def save(objects, path=None):
    path = path or SCENE_PATH
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump({"scanned_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "objects": objects}, f, indent=2, ensure_ascii=False)
    return path


def load(path=None):
    """读地图。没有或者坏了都返回 None, 由调用方决定怎么办。"""
    try:
        with open(path or SCENE_PATH) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_fresh(ttl_s, path=None):
    """读地图; 文件不存在、坏了、或比 ttl_s 旧, 都返回 None。

    过期地图比没有地图更糟 —— 东西被挪了而地图还记着老位置, 照着够会够到空处。
    所以调用方拿到 None 就该停, 而不是拿旧数据凑合。
    """
    path = path or SCENE_PATH
    try:
        if time.time() - os.path.getmtime(path) > ttl_s:
            return None
    except OSError:
        return None
    return load(path)


def find(scene, name, color=None):
    """按名字 (+颜色) 找一样东西, 找不到返回 None。

    同名的多个里取置信度最高的 —— 颜色能区分时优先按颜色过滤。
    """
    if not scene:
        return None
    hits = [o for o in scene.get("objects", [])
            if o["name"] == name and (color is None or o.get("color") == color)]
    return max(hits, key=lambda o: o["conf"]) if hits else None


def parse_target(phrase, color_vocab=None, noun_vocab=None):
    """把指令里的一串词拆成 ``(颜色, 名字)``。拆不动就整串当名字。

    规则 (按顺序):
      1. 首词在颜色表里, **且剩余部分在名词表里** → ``(首词, 剩余部分)``。
         ``"blue plate"`` → ``("blue", "plate")``
      2. 否则不拆, ``(None, 整串)``。
         ``"red fruit"`` → ``(None, "red fruit")``

    第 1 条里"剩余部分必须在名词表里"是关键。少了它, ``"red fruit"`` 会被拆成
    ``("red", "fruit")``, 然后拿 ``"fruit"`` 去查地图和 YOLO 什么都查不到 ——
    而 ``red fruit`` 本身是 YOLO-World 认得出的**单个类名**。

    多词颜色 (``"light blue"``) 暂不支持: 现在没有真实用例, 等出现了再说。
    """
    if not phrase:
        return None, phrase
    if color_vocab is None:
        color_vocab = COLOR_VOCAB
    if noun_vocab is None:
        noun_vocab = SCAN_VOCAB
    text = phrase.strip()
    low = text.lower()
    # 用前缀匹配而不是按空格切 —— 中文的颜色和名词之间没有空格 ("粉色盘子"),
    # 切词切不开。前缀匹配两种都吃, 代码还更短。
    # 长的颜色词优先: 免得短词抢在长词前面 (现在没有互为前缀的颜色词, 但顺序要定死)。
    for color in sorted(color_vocab, key=len, reverse=True):
        if low.startswith(color):
            rest = text[len(color):].strip()
            if rest and rest.lower() in noun_vocab:
                return color, rest
    return None, phrase


def detect_scene(perception, rgb, conf=YOLO_CONF_SCAN, framed_only=False):
    """跑一遍**建图用的那套**检测, 返回 ``[(名字, 颜色, 框, 置信度), ...]``。

    建图和"现场核对地图"必须用同一套判据。建图这边是 YOLO 认 SCAN_VOCAB 里的名词 +
    颜色从框内像素取; 核对若换成 ``detect_container`` 就会多一层 ``_well_framed``
    过滤, 把"框被画面边缘切掉"误报成"东西被挪了", 于是停下来问一个不存在的问题。

    ``framed_only=True`` (建图用): 贴画面边缘的框不算一次目击。框被切掉时框中心不是
    物体中心 (见 core/perception.py:_well_framed), 记进地图就是一个偏心的坐标; 更要命
    的是它会污染 ``seen_from`` —— 2026-10-06 复扫, 桃子在地图里是
    ``seen_from=['grasp2','grasp3']``, 而 grasp2 上桃子的框是 (522,388,640,480), 右边
    下边都切在画面外。``_poses_for`` 取 ``seen_from[0]``, 于是"最先看见它"的那个位姿
    恰好是只看到一角的那张, 抓取就照一个偏心的中心下爪。

    核对那边**不能**跟着严: 那儿只要确认东西"还在", 坐标本来就取地图记的, 框被画面
    切掉不算"东西被挪了"。
    """
    detections = perception.detect_objects(rgb, SCAN_VOCAB, conf=conf)
    if framed_only:
        detections = [d for d in detections if _well_framed(d, rgb.shape)]
    return [(SCAN_VOCAB[int(d[5])], dominant_color_name(rgb, d), d[:4], d[4])
            for d in detections]


def _world_point(T_base_to_cam, depth, box):
    """框中心的深度 -> 世界坐标。深度全空时返回 None。"""
    H, W = depth.shape
    x1, y1, x2, y2 = [int(v) for v in box]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(W, x2), min(H, y2)
    if x2 <= x1 or y2 <= y1:
        return None
    sub = depth[y1:y2, x1:x2]
    valid = sub[sub > 0]
    if not len(valid):
        return None
    point = pixel_to_camera_point2(
        np.array([[(x1 + x2) / 2, (y1 + y2) / 2]]),
        float(np.median(valid)) * 1e-3,
    ).flatten()
    T = np.eye(4)
    T[:3, 3] = point
    return (np.asarray(T_base_to_cam) @ T)[:3, 3].tolist()


def scan_workspace(move_to_pose, capture, transform, detect, poses):
    """从一个或多个观察位姿扫一遍工作台, 返回带世界坐标的记录。

    **不合并、不落盘** —— 那是调用方的事 (见 merge / save)。抽成函数而不是 Skill,
    是因为 CLAUDE.md 禁止流水线执行中跨 Skill 实例化 (新实例会另开 arm/hand 的 TCP
    连接, 破坏阶段衔接); 而 pick_and_place 也需要这段逻辑。

    回调契约 —— 每个只做一件事, 谁也别替谁做:

    | 回调 | 签名 | 契约 |
    |---|---|---|
    | ``move_to_pose`` | ``(name) -> None`` | 移到该观察位姿, **阻塞到到位** |
    | ``capture`` | ``(name) -> (rgb, depth)`` | 返回**当前**位姿下的一帧; name 只是给落盘命名用 |
    | ``transform`` | ``() -> (4,4)`` | **刷新并返回** "基座←相机" 变换 |
    | ``detect`` | ``(rgb) -> [(name, color, box, conf), ...]`` | 这一帧里认得的东西 |

    **``transform`` 必须每个位姿调一次**: ``T_base_to_cam`` 随臂动, 沿用上一个位姿会让
    所有世界坐标都算错, 而日志里看不出任何异常 (这个坑 look_around 踩过)。这里把它做成
    独立回调而不是塞进 ``capture``, 就是为了让"每帧刷新"在契约里显式存在 ——
    测试断言它的调用次数等于位姿数。

    自动路径**不跑 VLM**: VLM 那段的产出是给人读的场景描述, 对建图没用, 又慢又花钱。
    """
    out = []
    for pose in poses:
        move_to_pose(pose)
        T_base_to_cam = np.asarray(transform(), dtype=float)
        rgb, depth = capture(pose)
        for name, color, box, conf in detect(rgb):
            world = _world_point(T_base_to_cam, depth, box)
            if world is None:
                continue
            out.append({"name": name, "color": color, "world": world,
                        "conf": float(conf), "seen_from": [pose]})
    return out


def _self_check():
    """合并/查找的基本行为。"""
    e = lambda n, c, w, conf, pose: {                       # noqa: E731
        "name": n, "color": c, "world": w, "conf": conf, "seen_from": [pose]}
    merged = merge([
        e("plate", "pink", [-0.50, -0.36, 0.02], 0.92, "grasp1"),
        e("plate", "pink", [-0.52, -0.34, 0.02], 0.85, "grasp2"),   # 同一个, 2cm 外
        e("plate", "blue", [-0.42, 0.36, 0.01], 0.93, "grasp4"),    # 另一个盘子
    ])
    assert len(merged) == 2, merged
    pink = find({"objects": merged}, "plate", "pink")
    assert pink["seen_from"] == ["grasp1", "grasp2"], pink
    assert abs(pink["world"][0] - (-0.51)) < 1e-9, pink
    assert find({"objects": merged}, "plate", "green") is None
    assert find(None, "plate") is None

    # 工作台外的假框 (实测背景墙上那个 x=-1.135 的 "bottle") 不能进地图
    assert merge([e("bottle", "orange", [-1.135, 0.342, 0.037], 0.43, "grasp3")]) == []
    assert in_workspace([-0.51, -0.34, 0.02]) and not in_workspace([-1.135, 0.342, 0.037])

    # 拆词: 剩余部分不在名词表就不拆 —— 这条保住 "red fruit" 这种单类名
    assert parse_target("blue plate") == ("blue", "plate")
    assert parse_target("pink plate") == ("pink", "plate")
    assert parse_target("green bowl") == ("green", "bowl")
    assert parse_target("red fruit") == (None, "red fruit")   # "fruit" 不在名词表
    assert parse_target("orange") == (None, "orange")         # 既是颜色又是果子, 单词不拆
    assert parse_target("orange bowl") == ("orange", "bowl")
    assert parse_target("trash") == (None, "trash")           # 特殊模式词原样穿过
    print("scene._self_check ok")


if __name__ == "__main__":
    _self_check()
