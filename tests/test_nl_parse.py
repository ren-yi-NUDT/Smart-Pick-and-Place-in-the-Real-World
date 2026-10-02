"""run_nl.py 指令解析的静态检查（不碰硬件、不依赖 jev 服务）。

jev 调用用 stub 替掉，只验证关键词分支和兜底路径。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from run_nl import parse


def _stub(choice, conf=0.9):
    def ask(questions, state):
        return {"object": (choice, conf)}
    return ask


def test_container_keywords():
    assert parse("把香蕉扔垃圾桶", ask=_stub("banana"))["container"] == "trash"
    assert parse("把杯子放桌上", ask=_stub("cup"))["container"] == "desk"
    assert parse("把橙子递给我", ask=_stub("orange"))["container"] == "person"


def test_colored_plate_keeps_the_color():
    # "粉色盘子" 必须把颜色带进容器名，否则 YOLO 分不出是哪个盘子
    assert parse("把黄瓜放进粉色盘子", ask=_stub("cucumber"))["container"] == "pink plate"
    assert parse("把苹果放到碗里", ask=_stub("apple"))["container"] == "bowl"


def test_no_container_falls_back_to_grasp():
    plan = parse("抓个黄瓜", ask=_stub("cucumber"))
    assert plan["container"] is None and plan["object"] == "cucumber"


def test_look_around_needs_no_object():
    assert parse("看看桌上有什么", ask=_stub("none"))["skill"] == "look_around"


def test_jev_down_falls_back_to_glossary():
    def boom(questions, state):
        raise OSError("connection refused")
    plan = parse("把黄瓜递给我", ask=boom)
    assert plan["object"] == "cucumber" and plan["container"] == "person"
