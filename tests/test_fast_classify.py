import numpy as np
import pytest

from core import fast_classify


def test_bgr_is_converted_before_yolo(monkeypatch):
    class Perception:
        def detect_objects(self, image, names, conf):
            assert image[0, 0].tolist() == [255, 0, 0]
            return []

    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    frame[:, :, 2] = 255
    assert fast_classify.classify_frame(frame, Perception()) == []


class _Perception:
    def detect_objects(self, image, names, conf):
        return [[10, 20, 80, 100, 0.9, 0]]


def test_classify_frame_passes_fine_candidates_and_keeps_metadata(monkeypatch):
    seen = {}

    def fake_classify(hint, candidates):
        seen["candidates"] = candidates
        return "peach", 0.8, 0.9

    monkeypatch.setattr(fast_classify, "jev_classify", fake_classify)
    found = fast_classify.classify_frame(
        np.zeros((120, 120, 3), dtype=np.uint8), _Perception())
    assert seen["candidates"] == fast_classify.FINE_CANDIDATES
    assert found == [{
        "box": [10, 20, 80, 100], "coarse": "fruit", "class": "peach",
        "prob": 0.8, "confidence": 0.9,
    }]


def test_classify_frame_rejects_jev_failure_instead_of_falling_back(monkeypatch):
    monkeypatch.setattr(
        fast_classify, "jev_classify",
        lambda *args, **kwargs: (_ for _ in ()).throw(ValueError("bad response")))
    assert fast_classify.classify_frame(
        np.zeros((120, 120, 3), dtype=np.uint8), _Perception()) == []


def test_jev_classify_rejects_unknown_choice_and_invalid_confidence(monkeypatch):
    class _Response:
        def __enter__(self):
            return self
        def __exit__(self, *args):
            return False
        def read(self):
            return (b'{"answers":{"class":{"choice":"not-a-candidate",'
                    b'"probabilities":{"not-a-candidate":0.9},"confidence":0.9}}}')

    monkeypatch.setattr(fast_classify.urllib.request, "urlopen", lambda *a, **k: _Response())
    with pytest.raises(ValueError, match="choice"):
        fast_classify.jev_classify("hint", candidates=["apple"])

    class _BadConfidence(_Response):
        def read(self):
            return (b'{"answers":{"class":{"choice":"apple",'
                    b'"probabilities":{"apple":0.9},"confidence":2}}}')

    monkeypatch.setattr(fast_classify.urllib.request, "urlopen", lambda *a, **k: _BadConfidence())
    with pytest.raises(ValueError, match="confidence"):
        fast_classify.jev_classify("hint", candidates=["apple"])
