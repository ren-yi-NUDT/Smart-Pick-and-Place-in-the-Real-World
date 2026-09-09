from skills.grasp import GraspSkill


def test_grasp_skill_retries_visual_grasp_after_failure():
    skill = GraspSkill.__new__(GraspSkill)
    calls = []

    def fake_visual_grasp(object_name, **kwargs):
        calls.append((object_name, kwargs))
        return len(calls) == 2

    skill.visual_grasp = fake_visual_grasp

    assert skill.execute(object="orange", side="left") is True
    assert len(calls) == 2
    assert calls[0][0] == calls[1][0] == "orange"
    assert calls[0][1] == calls[1][1]


def test_grasp_skill_honors_max_attempts():
    skill = GraspSkill.__new__(GraspSkill)
    calls = []
    skill.visual_grasp = lambda object_name, **kwargs: calls.append(1) or False

    assert skill.execute(object="orange", max_attempts=3) is False
    assert len(calls) == 3
