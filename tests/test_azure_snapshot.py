import time

import numpy as np
import pytest

from core.azure_vision import VisionOwner, validate_snapshot


class FakeCamera:
    def __init__(self):
        self.packet = {
            "session_id": "s1", "frame_id": 1,
            "captured_at": time.monotonic(),
            "bgr": np.zeros((100, 120, 3), dtype=np.uint8),
        }

    def get_snapshot(self, max_age=0.5):
        return dict(self.packet)


def test_owner_publishes_frame_metadata_and_tracks_same_class_box():
    camera = FakeCamera()
    owner = VisionOwner(camera, lambda image: [[10, 20, 50, 60, 0.8, 0]])
    first = owner.update()
    camera.packet["frame_id"] = 2
    second = owner.update()

    assert first["session_id"] == "s1"
    assert first["frame_id"] == 1
    assert first["width"] == 120 and first["height"] == 100
    assert first["objects"][0]["target_id"] == second["objects"][0]["target_id"]
    assert first["objects"][0]["bbox"] == [10.0, 20.0, 50.0, 60.0]


def test_owner_does_not_match_different_classes():
    camera = FakeCamera()
    detections = [[[10, 20, 50, 60, 0.8, 0]], [[10, 20, 50, 60, 0.8, 1]]]
    owner = VisionOwner(camera, lambda image: detections.pop(0))

    first = owner.update()
    camera.packet["frame_id"] = 2
    second = owner.update()

    assert second["objects"][0]["target_id"] != first["objects"][0]["target_id"]


def test_owner_invalidates_ids_for_ambiguous_iou_matches():
    camera = FakeCamera()
    detections = [
        [[0, 0, 40, 40, 0.8, 0], [30, 0, 70, 40, 0.8, 0]],
        [[15, 0, 55, 40, 0.8, 0]],
    ]
    owner = VisionOwner(camera, lambda image: detections.pop(0))

    first = owner.update()
    camera.packet["frame_id"] = 2
    second = owner.update()

    assert second["objects"][0]["target_id"] not in {
        first["objects"][0]["target_id"], first["objects"][1]["target_id"]
    }


def test_owner_rejects_invalid_detector_boxes_and_uses_unique_restart_provenance():
    camera = FakeCamera()
    owner = VisionOwner(camera, lambda image: [
        [float("nan"), 0, 10, 10, 0.8, 0],
        [0, 0, 10, 10, float("inf"), 0],
        [0, 0, 10, 10, 0.8, 0],
        [0, 0, 10, 10, 0.8, 1],
    ])
    snapshot = owner.update()

    assert len(snapshot["objects"]) == 2
    assert len({obj["target_id"] for obj in snapshot["objects"]}) == 2
    assert all(np.isfinite(obj["confidence"]) for obj in snapshot["objects"])

    restarted = VisionOwner(camera, lambda image: [[0, 0, 10, 10, 0.8, 0]])
    assert restarted.update()["objects"][0]["target_id"] not in {
        obj["target_id"] for obj in snapshot["objects"]
    }


def test_owner_returns_immutable_nested_snapshot_copies():
    camera = FakeCamera()
    owner = VisionOwner(camera, lambda image: [[10, 20, 50, 60, 0.8, 0]])
    returned = owner.update()
    returned["objects"][0]["bbox"][0] = 999

    assert owner.snapshot()["objects"][0]["bbox"] == [10.0, 20.0, 50.0, 60.0]


def test_validate_snapshot_rejects_stale_or_malformed_snapshot():
    snapshot = {
        "session_id": "s1", "frame_id": 1,
        "captured_at": time.monotonic() - 10,
        "width": 120, "height": 100, "objects": [],
    }
    with pytest.raises(ValueError, match="stale"):
        validate_snapshot(snapshot, max_age=1)

    snapshot["captured_at"] = time.monotonic()
    snapshot["objects"] = [{"target_id": "x", "bbox": [1, 2, 3],
                             "confidence": 0.9}]
    with pytest.raises(ValueError, match="bbox"):
        validate_snapshot(snapshot, max_age=1)


def test_validate_snapshot_rejects_nonfinite_detection_metadata():
    snapshot = {
        "session_id": "s1", "frame_id": 1, "captured_at": time.monotonic(),
        "width": 120, "height": 100,
        "objects": [{"target_id": "x", "bbox": [1, 2, float("inf"), 4],
                      "confidence": 0.9}],
    }
    with pytest.raises(ValueError, match="bbox"):
        validate_snapshot(snapshot, max_age=1)

    snapshot["objects"][0]["bbox"] = [1, 2, 3, 4]
    snapshot["objects"][0]["confidence"] = float("nan")
    with pytest.raises(ValueError, match="confidence"):
        validate_snapshot(snapshot, max_age=1)
