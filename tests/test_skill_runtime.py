import os

from skills.base import Skill
from core.skill_runtime import RobotContext, SkillResult


class _SuccessfulSkill(Skill):
    def execute(self, **kwargs):
        return {"ok": True, "value": kwargs.get("value", 1)}


class _LegacySkill(Skill):
    # This represents an external/local Skill that has not been migrated yet.
    def run(self, **kwargs):
        return True


class _FailingSkill(Skill):
    def execute(self, **kwargs):
        raise RuntimeError("simulated failure")


def _save_path(tmp_path):
    return os.path.join(str(tmp_path), "skill-log")


def test_skill_lifecycle_normalizes_result_and_exposes_context(tmp_path):
    skill = _SuccessfulSkill(save_path=_save_path(tmp_path))

    result = skill.run(value=7)

    assert isinstance(result, SkillResult)
    assert result.ok is True
    assert result.code == "SUCCESS"
    assert result.data["value"] == 7
    assert skill.lifecycle_state == "finished"
    assert skill.last_result is result
    assert isinstance(skill.context, RobotContext)
    assert skill.context.config is skill.config


def test_legacy_run_override_is_adapted_to_execute(tmp_path):
    skill = _LegacySkill(save_path=_save_path(tmp_path))

    result = skill.run()

    assert isinstance(result, SkillResult)
    assert result.ok is True
    assert _LegacySkill.execute is not None


def test_exceptions_become_failed_results(tmp_path):
    skill = _FailingSkill(save_path=_save_path(tmp_path))

    result = skill.run()

    assert isinstance(result, SkillResult)
    assert result.ok is False
    assert result.code == "EXCEPTION"
    assert "simulated failure" in result.message


def test_success_key_is_normalized_without_hiding_failure(tmp_path):
    result = _SuccessfulSkill(save_path=_save_path(tmp_path)).run()
    assert result.ok is True

    from core.skill_runtime import normalize_result
    failed = normalize_result({"success": False, "message": "bridge failed"})
    assert failed.ok is False
    assert failed.code == "FAILED"
