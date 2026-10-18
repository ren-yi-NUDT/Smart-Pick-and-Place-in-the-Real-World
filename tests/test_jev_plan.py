import time

import pytest

from core.jev_command import build_plan, validate_plan


def snapshot(objects, age=0):
    return {"session_id": "s", "frame_id": 4,
            "captured_at": time.monotonic() - age,
            "width": 640, "height": 480, "objects": objects}


def obj(target_id, name, confirmed=True, confidence=0.9):
    return {"target_id": target_id, "name": name, "color": "green",
            "bbox": [1, 2, 20, 30], "confidence": confidence,
            "confirmed": confirmed}


def test_build_plan_selects_only_fixed_action_and_target_ids():
    calls = []

    def ask(state, candidates):
        calls.append((state, candidates))
        if candidates == ["grasp", "pick_and_place", "look_around", "none"]:
            return "pick_and_place", 0.9, 0.9
        if candidates == ["s1:object"]:
            return "s1:object", 0.9, 0.9
        return "s1:plate", 0.9, 0.9

    plan = build_plan("把物体放进盘子", snapshot([
        obj("s1:object", "cucumber"), obj("s1:plate", "plate")]), ask=ask)
    assert plan["action"] == "pick_and_place"
    assert plan["object_target_id"] == "s1:object"
    assert plan["container_target_id"] == "s1:plate"
    assert len(calls) == 3


def test_validate_plan_rejects_stale_or_unconfirmed_target():
    current = snapshot([obj("s:obj", "apple")])
    plan = {"action": "grasp", "object_target_id": "s:obj",
            "container_target_id": None, "probability": .9, "confidence": .9,
            "session_id": "s", "frame_id": 4,
            "captured_at": current["captured_at"]}
    assert validate_plan(plan, current)
    current["objects"][0]["confirmed"] = False
    with pytest.raises(ValueError, match="confirmed"):
        validate_plan(plan, current)


def test_validate_plan_rejects_future_snapshot_and_invalid_score():
    current = snapshot([obj("s:obj", "apple")])
    future = dict(current, captured_at=time.monotonic() + 1)
    plan = {"action": "grasp", "object_target_id": "s:obj",
            "container_target_id": None, "probability": .9, "confidence": .9,
            "session_id": "s", "frame_id": 4,
            "captured_at": future["captured_at"]}
    with pytest.raises(ValueError, match="stale"):
        validate_plan(plan, future)
    plan["captured_at"] = current["captured_at"]
    plan["confidence"] = float("nan")
    with pytest.raises(ValueError, match="confidence"):
        validate_plan(plan, current)
