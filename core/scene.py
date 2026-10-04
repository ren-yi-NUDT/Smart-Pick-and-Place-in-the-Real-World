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

SCENE_PATH = "./log/scene.json"

# 扫描词表。一次全给 YOLO —— 实测亮图上一遍就能出结果, 不用逐类跑 (逐类慢 15 倍)。
SCAN_VOCAB = [
    "plate", "bowl", "cup", "bottle", "apple", "orange", "banana",
    "peach", "pear", "lemon", "cucumber", "dragon fruit", "kiwi fruit",
    "toy", "box",
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
    """把多个视角的记录并成一份; 同一个 (名字, 颜色) 且位置相近的取平均。

    工作台外的记录直接丢掉 —— 那些是背景上的假框, 留着会让人按地图去够一个
    根本不在桌上的坐标。
    """
    out = []
    for e in entries:
        if not in_workspace(e["world"]):
            continue
        for o in out:
            if (o["name"] == e["name"] and o.get("color") == e.get("color")
                    and _dist(o["world"], e["world"]) < tol):
                n = len(o["seen_from"])
                o["world"] = ((np.array(o["world"]) * n + np.array(e["world"]))
                              / (n + 1)).tolist()
                o["seen_from"].append(e["seen_from"][0])
                o["conf"] = max(o["conf"], e["conf"])
                break
        else:
            out.append(dict(e))
    return out


def save(objects, path=SCENE_PATH):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump({"scanned_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "objects": objects}, f, indent=2, ensure_ascii=False)
    return path


def load(path=SCENE_PATH):
    """读地图。没有或者坏了都返回 None, 由调用方决定怎么办。"""
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def find(scene, name, color=None):
    """按名字 (+颜色) 找一样东西, 找不到返回 None。

    同名的多个里取置信度最高的 —— 颜色能区分时优先按颜色过滤。
    """
    if not scene:
        return None
    hits = [o for o in scene.get("objects", [])
            if o["name"] == name and (color is None or o.get("color") == color)]
    return max(hits, key=lambda o: o["conf"]) if hits else None


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
    print("scene._self_check ok")


if __name__ == "__main__":
    _self_check()
