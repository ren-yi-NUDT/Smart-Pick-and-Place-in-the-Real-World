"""场景地图: 合并/查找/存取, 以及 look_around 存 / pick_and_place 查的接线。"""
import ast
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

from core import scene


def _e(name, color, world, conf, pose):
    return {"name": name, "color": color, "world": world, "conf": conf, "seen_from": [pose]}


def test_merge_averages_same_object_and_keeps_distinct_ones():
    merged = scene.merge([
        _e("plate", "pink", [-0.50, -0.36, 0.02], 0.92, "grasp1"),
        _e("plate", "pink", [-0.52, -0.34, 0.02], 0.85, "grasp2"),  # 同一个, 2cm 外
        _e("plate", "blue", [-0.42, 0.36, 0.01], 0.93, "grasp4"),   # 另一个盘子
        _e("orange", "orange", [-0.51, -0.35, 0.03], 0.65, "grasp2"),
    ])
    assert len(merged) == 3, merged
    pink = scene.find({"objects": merged}, "plate", "pink")
    assert pink["seen_from"] == ["grasp1", "grasp2"], pink
    assert abs(pink["world"][0] - (-0.51)) < 1e-9, pink
    assert pink["conf"] == 0.92, "置信度取高值, 不是平均"


def test_merge_ignores_color_so_one_object_stays_one_entry():
    """同一个东西换个视角被叫成另一个颜色, 不能变成两条记录。

    2026-10-06 桃子实测: grasp2 看是 pink apple (0.88), grasp3 看是 yellow apple
    (0.85), 坐标差 19mm。分开成两条后, find 取置信度高的那条 (grasp2), 而那条只带
    grasp2 一个观察位姿, 偏偏 grasp2 上 AnyGrasp 给不出落在桃子上的候选 —— 抓取失败。
    """
    merged = scene.merge([
        _e("apple", "pink", [-0.382, 0.167, 0.058], 0.88, "grasp2"),
        _e("apple", "yellow", [-0.390, 0.184, 0.067], 0.85, "grasp3"),
    ])
    assert len(merged) == 1, merged
    assert merged[0]["seen_from"] == ["grasp2", "grasp3"]
    assert merged[0]["color"] == "pink", "颜色跟看得最准的那次走"
    # 同名但离得远的仍然是两样东西
    assert len(scene.merge([
        _e("apple", "pink", [-0.382, 0.167, 0.058], 0.88, "grasp2"),
        _e("apple", "yellow", [-0.500, -0.300, 0.060], 0.85, "grasp1"),
    ])) == 2


def test_find_requires_color_match_and_survives_missing_scene():
    merged = scene.merge([_e("plate", "pink", [-0.50, -0.36, 0.02], 0.92, "grasp1")])
    s = {"objects": merged}
    assert scene.find(s, "plate", "pink") is not None
    assert scene.find(s, "plate", "green") is None, "颜色对不上宁可返回 None, 不能放错盘子"
    assert scene.find(s, "bowl") is None
    assert scene.find(None, "plate") is None
    assert scene.find({"objects": []}, "plate") is None


def test_save_load_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setattr(scene, "SCENE_PATH", str(tmp_path / "scene.json"))
    path = scene.save([_e("cup", "white", [0.1, 0.2, 0.03], 0.7, "grasp3")])
    loaded = scene.load(path)
    assert loaded["objects"][0]["name"] == "cup"
    assert "scanned_at" in loaded
    # 坏文件 / 不存在都不能抛, 由调用方决定
    (tmp_path / "broken.json").write_text("{not json")
    assert scene.load(str(tmp_path / "broken.json")) is None
    assert scene.load(str(tmp_path / "nope.json")) is None


def test_scan_vocab_has_no_duplicates():
    assert len(scene.SCAN_VOCAB) == len(set(scene.SCAN_VOCAB))


@pytest.mark.parametrize("rel,needle", [
    ("skills/look_around.py", "scene_map.save"),
    ("skills/pick_and_place.py", "_placing_position_from_scene"),
])
def test_scene_map_is_wired_into_the_skills(rel, needle):
    """扫描端不存 / 抓放端不查, 地图就只是个没人读的文件。"""
    assert needle in (ROOT / rel).read_text()


def test_detect_scene_maps_class_index_to_vocab_name(monkeypatch):
    """建图和现场核对共用这一个检测 —— 名字和颜色都得从同一个地方来。"""
    class _P:
        def detect_objects(self, rgb, names, conf=None):
            assert names == scene.SCAN_VOCAB          # 一次全给, 不逐类跑
            return [[1, 2, 3, 4, 0.5, scene.SCAN_VOCAB.index("plate")]]

    monkeypatch.setattr(scene, "dominant_color_name", lambda rgb, d: "blue")
    assert scene.detect_scene(_P(), None) == [("plate", "blue", [1, 2, 3, 4], 0.5)]


def test_framed_only_drops_boxes_clipped_by_the_frame(monkeypatch):
    """建图只收完整落在画面里的框。贴边的框中心不是物体中心, 记进地图是个偏心坐标,
    而且会让 seen_from[0] 变成"只看到一角"的那张 —— _poses_for 就照它去抓。"""
    import numpy as np

    class _P:
        def detect_objects(self, rgb, names, conf=None):
            return [[10, 20, 100, 200, 0.9, scene.SCAN_VOCAB.index("plate")],      # 完整
                    [522, 388, 640, 480, 0.6, scene.SCAN_VOCAB.index("apple")]]    # 贴右+下

    monkeypatch.setattr(scene, "dominant_color_name", lambda rgb, d: "blue")
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    assert [d[0] for d in scene.detect_scene(_P(), rgb)] == ["plate", "apple"]
    assert [d[0] for d in scene.detect_scene(_P(), rgb, framed_only=True)] == ["plate"]


def test_look_around_refreshes_the_transform_per_pose():
    """T_base_to_cam 随臂动, 每个位姿都得重新取 —— 沿用上一个位姿会让世界坐标全错。"""
    src = (ROOT / "skills/look_around.py").read_text()
    assert src.count("save_current_transformation()") >= 1
    # 必须在循环体内 (缩进 12 空格 = for 循环里)
    assert any(line.strip() == "self.save_current_transformation()"
               and len(line) - len(line.lstrip()) >= 12
               for line in src.splitlines())


# ---------------------------------------------------------------------------
# parse_target: 把指令里的一串词拆成 (颜色, 名字)
# ---------------------------------------------------------------------------
def test_parse_target_splits_color_from_noun():
    assert scene.parse_target("blue plate") == ("blue", "plate")
    assert scene.parse_target("pink plate") == ("pink", "plate")
    assert scene.parse_target("green bowl") == ("green", "bowl")


def test_parse_target_refuses_to_split_when_remainder_is_not_a_noun():
    """`red fruit` 是 YOLO-World 认得出的**单个类名**, 拆成 fruit 就什么都查不到了。"""
    assert scene.parse_target("red fruit") == (None, "red fruit")


def test_parse_target_handles_words_that_are_both_color_and_noun():
    assert scene.parse_target("orange") == (None, "orange")        # 单词: 不拆
    assert scene.parse_target("orange bowl") == ("orange", "bowl")  # 后面有名词: 拆


def test_parse_target_passes_unknown_phrases_through():
    """特殊模式词 (trash / person / desk) 和查不到的词原样穿过, 由调用方决定怎么办。"""
    assert scene.parse_target("trash") == (None, "trash")
    assert scene.parse_target("person") == (None, "person")
    assert scene.parse_target("desk") == (None, "desk")
    assert scene.parse_target("") == (None, "")
    assert scene.parse_target(None) == (None, None)


def test_parse_target_handles_chinese_without_spaces():
    """中文的颜色和名词之间没有空格, 所以是前缀匹配不是按空格切。"""
    assert scene.parse_target("粉色盘子", noun_vocab=["盘子"]) == ("粉色", "盘子")


# ---------------------------------------------------------------------------
# scan_workspace: 共享扫描函数 (look_around 和 pick_and_place 共用, 不 new Skill)
# ---------------------------------------------------------------------------
import numpy as np          # noqa: E402


def _fake_scan(poses, detections=None, depth_mm=1000, T=None):
    """跑一次 scan_workspace, 返回 (entries, 调用流水)。"""
    calls = []
    detections = detections or [("plate", "pink", [100, 100, 200, 200], 0.9)]
    T = np.eye(4) if T is None else T

    def move_to_pose(name):
        calls.append(("move", name))

    def capture(name):
        calls.append(("capture", name))
        return (np.zeros((480, 640, 3), np.uint8),
                np.full((480, 640), depth_mm, np.uint16))

    def transform():
        calls.append(("transform", None))
        return T

    def detect(rgb):
        calls.append(("detect", None))
        return detections

    entries = scene.scan_workspace(move_to_pose, capture, transform, detect, poses)
    return entries, calls


def test_scan_workspace_refreshes_transform_once_per_pose():
    """漏调一次 transform, 世界坐标就全错 —— 而日志里看不出来。所以钉住调用次数。"""
    poses = ["grasp1", "grasp2", "grasp3"]
    _, calls = _fake_scan(poses)
    assert [c for c in calls if c[0] == "transform"] == [("transform", None)] * len(poses)


def test_scan_workspace_visits_poses_in_order():
    poses = ["grasp1", "grasp4"]
    _, calls = _fake_scan(poses)
    assert [c[1] for c in calls if c[0] == "move"] == poses
    # 每个位姿: 先到位, 再刷新变换, 再拍照 —— 顺序错了坐标就是上一个位姿的
    assert [c[0] for c in calls] == ["move", "transform", "capture", "detect"] * len(poses)


def test_scan_workspace_returns_unmerged_unsaved_entries():
    """同一个东西在两个位姿看到就是 2 条 —— 合并/落盘是调用方的事。"""
    entries, _ = _fake_scan(["grasp1", "grasp2"])
    assert len(entries) == 2
    assert all(set(e) == {"name", "color", "world", "conf", "seen_from"} for e in entries)
    assert [e["seen_from"] for e in entries] == [["grasp1"], ["grasp2"]]


def test_scan_workspace_skips_detections_with_no_depth():
    """深度全是 0 的框算不出世界坐标, 直接丢, 不能塞个假坐标进地图。"""
    entries, _ = _fake_scan(["grasp1"], depth_mm=0)
    assert entries == []
