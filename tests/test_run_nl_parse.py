"""run_nl.parse 的多物体解析检查。"""
import sys

sys.path.insert(0, ".")

from run_nl import parse, split_sequence


def _no_jev(*_a, **_k):
    return {"object": ("none", 0.0)}


def test_multi_object_collects_all_gloss_nouns_in_order():
    plan = parse("把苹果，柠檬和橙子全部放到蓝色盘子里", ask=_no_jev)
    assert plan["objects"] == ["apple", "lemon", "orange"]
    assert plan["container"] == "blue plate"


def test_single_object_still_uses_jev():
    plan = parse("把黄瓜放进粉色盘子",
                 ask=lambda q, s: {"object": ("cucumber", 0.9)})
    assert "objects" not in plan
    assert plan["object"] == "cucumber"


def test_substring_nouns_do_not_double_match():
    plan = parse("把瓶子放到桌上", ask=_no_jev)
    assert "objects" not in plan
    assert plan["object"] == "bottle"  # "瓶" 不能和 "瓶子" 重复计入


def test_unknown_word_learns_from_scene_map(monkeypatch):
    """词表外的物体名 -> 问 jev 桌上哪个已检测物体 -> conf 够就学。"""
    import core.object_vocab as ov
    added = []
    monkeypatch.setattr(ov, "add", lambda zh, en: added.append((zh, en)) or True)

    def ask(q, s):
        # 第一问(词表内抽取)答 none, 第二问(学习匹配)答 pitaya
        if "none" in q["object"]["criteria"]:
            return {"object": ("none", 0.9)}
        return {"object": ("pitaya", 0.85)}

    plan = parse("把牛油果放进蓝色盘子", ask=ask,
                 scene_names=["pitaya", "plate"])
    assert plan["object"] == "pitaya"
    assert plan["learned_from"] == "牛油果"
    assert added == [("牛油果", "pitaya")]


def test_unknown_word_low_conf_not_learned(monkeypatch):
    import core.object_vocab as ov
    added = []
    monkeypatch.setattr(ov, "add", lambda zh, en: added.append((zh, en)) or True)

    plan = parse("把牛油果放进蓝色盘子",
                 ask=lambda q, s: {"object": ("none", 0.9)},
                 scene_names=["pitaya"])
    assert plan["object"] is None
    assert added == []


def test_jev_candidates_and_gloss_derive_from_vocab(monkeypatch):
    """候选词表应随 object_vocab.json 走（scan ∪ gloss 值）。"""
    import core.object_vocab as ov
    monkeypatch.setattr(ov, "load", lambda: (["durian"], {"榴莲": "durian"}))
    seen = {}

    def ask(q, s):
        seen["criteria"] = list(q["object"]["criteria"])
        return {"object": ("durian", 0.9)}

    plan = parse("把榴莲放进蓝色盘子", ask=ask)
    assert "durian" in seen["criteria"]
    assert plan["object"] == "durian"


# --- 动作序列切分 ---

def test_sequence_splits_on_connectors():
    assert split_sequence("把苹果放进蓝盘子然后把香蕉递给我") == \
        ["把苹果放进蓝盘子", "把香蕉递给我"]
    assert split_sequence("看看桌上有什么再把橙子放到桌上") == \
        ["看看桌上有什么", "把橙子放到桌上"]


def test_sequence_comma_split_needs_action_on_both_sides():
    assert split_sequence("把苹果放进蓝盘子，把香蕉递给我") == \
        ["把苹果放进蓝盘子", "把香蕉递给我"]
    # 多物体句不能被逗号误拆
    assert split_sequence("把苹果，柠檬和橙子全部放到蓝色盘子里") == \
        ["把苹果，柠檬和橙子全部放到蓝色盘子里"]


def test_sequence_single_action_untouched():
    assert split_sequence("把黄瓜放进粉色盘子") == ["把黄瓜放进粉色盘子"]


def test_sequence_each_segment_parses_independently():
    segs = split_sequence("把苹果放进蓝盘子然后把香蕉递给我")
    plans = [parse(s, ask=lambda q, s: {"object": ("none", 0.0)}) for s in segs]
    assert plans[0]["skill"] == "pick_and_place"
    assert plans[0]["object"] == "apple"
    assert plans[0]["container"] == "blue plate"
    # 递给用户走 pick_and_place(container="person") 递送模式, 不是 fetch_from_user
    assert plans[1]["skill"] == "pick_and_place"
    assert plans[1]["object"] == "banana"
    assert plans[1]["container"] == "person"
