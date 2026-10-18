import numpy as np

from core import scene


class FakePerception:
    def detect_objects(self, image, names, conf):
        assert conf < 0.1
        return [
            [100.0, 100.0, 220.0, 220.0, 0.02, scene.SCAN_VOCAB.index("apple")],
            [300.0, 100.0, 500.0, 300.0, 0.8, scene.SCAN_VOCAB.index("plate")],
        ]


def test_canonical_names_share_user_and_detector_vocab():
    assert scene.canonical_object_name("dragon fruit") == "pitaya"
    assert scene.canonical_object_name("dragonfruit") == "pitaya"
    assert scene.canonical_object_name("桃子") == "peach"


def test_vlm_semantics_replace_weak_yolo_labels_and_color():
    class VLM:
        def analyze(self, image, prompt, max_tokens, temperature):
            assert max_tokens == 8192
            assert "100" in prompt and "pitaya" in prompt
            return '{"objects":[{"index":0,"name":"dragon fruit","color":"pink"}]}'

    found = scene.detect_scene(FakePerception(), np.zeros((480, 640, 3), dtype=np.uint8),
                               vlm=VLM())
    assert len(found) == 1, "未被 VLM 确认的框不能进入结果"
    assert found[0][0:2] == ("pitaya", "pink")
    assert found[0][2] == [100.0, 100.0, 220.0, 220.0]
    assert found[0][3] == 0.02


def test_invalid_vlm_response_stops_instead_of_using_weak_boxes():
    class VLM:
        def analyze(self, *args, **kwargs):
            return "not json"

    import pytest
    with pytest.raises(ValueError, match="VLM"):
        scene.detect_scene(FakePerception(), np.zeros((480, 640, 3), dtype=np.uint8),
                           vlm=VLM())
