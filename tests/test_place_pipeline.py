import numpy as np

from skills.place import PlaceSkill


class _Config:
    shared = {}

    def get_camera_intrinsics(self, side):
        return {"fx": 385.0, "fy": 385.0, "cx": 320.0, "cy": 240.0}

    def get_grasp_scoring(self, side):
        return {}


def _skill_with_rgbd():
    """Return (skill, place_pipeline) with RGB-D fixtures seeded on the skill.

    Pipeline methods read skill state through delegation, so fixtures and
    lazy-property seeds (_vlm/_perception) must live on the skill, while
    pipeline methods are patched/called on the pipeline itself.
    """
    skill = PlaceSkill.__new__(PlaceSkill)
    skill.config = _Config()
    skill.rgb = np.zeros((120, 160, 3), dtype=np.uint8)
    skill.depth = np.full((120, 160), 700, dtype=np.uint16)
    skill._placement_side = "left"
    return skill, skill.place_pipeline


def test_placement_execution_offset_defaults_to_zero():
    _, pp = _skill_with_rgbd()

    np.testing.assert_array_equal(
        pp._placement_execution_offset_base_m("left"), np.zeros(3)
    )


def test_place_samples_safe_interior_points_using_object_footprint():
    skill, pp = _skill_with_rgbd()
    pp._get_side_transforms = lambda side: (np.eye(4), np.eye(4))
    pp._select_best_container_grasp = lambda *args, **kwargs: {
        "rotation": np.eye(3), "index": 2, "score": 0.9
    }

    target = pp._placing_position_from_detections(
        "pink bowl", [[20, 20, 140, 100, 1.0, 0.0]],
        side="left", source="test", object_size_m=0.04,
    )

    assert target.shape == (4, 4)
    assert len(pp._placement_candidate_targets_world) >= 2
    assert np.count_nonzero(pp._placement_region_mask) > 0


def test_vlm_box_is_tried_before_direct_detector():
    skill, pp = _skill_with_rgbd()

    class _Vlm:
        def ground_object(self, image, name):
            return {"prompts": [name], "box": [20, 20, 140, 100], "found": True}

    class _Perception:
        def detect_objects(self, image, prompts, conf=0.25):
            return [[21, 21, 139, 99, 0.8, 0.0]]

    skill._vlm = _Vlm()
    skill._perception = _Perception()
    calls = []

    def fake_position(name, detections, side="left", source="YOLO", object_size_m=None):
        calls.append(source)
        return None

    pp._placing_position_from_detections = fake_position
    result = pp._get_placing_position("blue plate", skill.rgb, side="left")

    assert result == []
    assert calls[0] == "VLM target box"


def test_missing_object_name_keeps_backward_compatible_release_policy():
    skill, pp = _skill_with_rgbd()
    skill.config.shared = {}
    skill.get_camera_obs = lambda side="left": (skill.rgb, skill.depth)
    skill._perception = type(
        "P", (), {"detect_objects": lambda self, image, names, conf=0.2: [[10, 10, 150, 110, 1.0, 0.0]]}
    )()

    assert pp._verify_object_in_container(None, "blue plate", side="left") is True
