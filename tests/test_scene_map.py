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


def test_look_around_refreshes_the_transform_per_pose():
    """T_base_to_cam 随臂动, 每个位姿都得重新取 —— 沿用上一个位姿会让世界坐标全错。"""
    src = (ROOT / "skills/look_around.py").read_text()
    assert src.count("save_current_transformation()") >= 1
    # 必须在循环体内 (缩进 12 空格 = for 循环里)
    assert any(line.strip() == "self.save_current_transformation()"
               and len(line) - len(line.lstrip()) >= 12
               for line in src.splitlines())
