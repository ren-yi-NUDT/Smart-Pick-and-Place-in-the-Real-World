from types import SimpleNamespace

import numpy as np

from core.grasp_pipeline import GraspPipeline


class _Config:
    def get_grasp_scoring(self, side):
        return {
            "weights": {
                "anygrasp": 0.40,
                "twin": 0.30,
                "width": 0.15,
                "length": 0.05,
                "angle": 0.10,
            },
            "preferred_approach_axis_base": [0.0, 0.0, 1.0],
        }


def _candidate(index, anygrasp_score):
    return {
        "index": index,
        "pose": np.eye(4),
        "anygrasp_normalized": anygrasp_score,
        "width_m": 0.045,
        "height_m": 0.03,
    }


def test_lazy_twin_planning_stops_at_best_reachable_candidate():
    pipeline = GraspPipeline(SimpleNamespace(config=_Config()))
    candidates = [
        _candidate(0, 1.0),
        _candidate(1, 0.8),
        _candidate(2, 0.6),
    ]
    calls = []

    def fake_plan(candidate, side, obs_pose):
        calls.append(candidate["index"])
        candidate["twin_reachable"] = float(candidate["index"] == 1)
        return bool(candidate["twin_reachable"])

    pipeline._plan_grasp_candidate = fake_plan

    selected, ranked, planned_count = pipeline._plan_best_grasp_candidate(
        candidates, "left", {}
    )

    assert selected["index"] == 1
    assert calls == [0, 1]
    assert planned_count == 2
    assert "twin_reachable" not in candidates[2]
    assert ranked[0]["index"] == 1
