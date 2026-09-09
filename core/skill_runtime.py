#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Common runtime contracts for robot skills.

The project historically used a mixture of ``bool``, ``None`` and arbitrary
objects as skill return values.  This module provides a small, dependency-free
runtime contract without coupling task code to a particular robot driver.
"""

from dataclasses import dataclass, field
from datetime import datetime
import uuid


@dataclass
class SkillResult:
    """Normalized result of a skill execution.

    ``SkillResult`` remains truthy/falsy like the old boolean API, so existing
    composite skills can keep using ``if not result`` while callers gain an
    error code and structured payload.
    """

    ok: bool
    code: str
    message: str = ""
    data: object = None
    recoverable: bool = False
    run_id: str = ""
    metadata: dict = field(default_factory=dict)

    def __bool__(self):
        return self.ok

    def to_dict(self):
        result = {
            "ok": self.ok,
            "code": self.code,
            "message": self.message,
            "recoverable": self.recoverable,
        }
        if self.run_id:
            result["run_id"] = self.run_id
        if self.data is not None:
            result["data"] = _json_safe(self.data)
        if self.metadata:
            result["metadata"] = _json_safe(self.metadata)
        return result


def normalize_result(value, run_id=""):
    """Convert legacy skill return values to :class:`SkillResult`."""
    if isinstance(value, SkillResult):
        if run_id and not value.run_id:
            value.run_id = run_id
        return value

    if isinstance(value, bool):
        return SkillResult(
            ok=value,
            code="SUCCESS" if value else "FAILED",
            message="Skill execution succeeded" if value else "Skill execution failed",
            run_id=run_id,
        )

    if value is None:
        return SkillResult(
            ok=False,
            code="NO_RESULT",
            message="Skill returned no result",
            run_id=run_id,
        )

    if isinstance(value, dict):
        # Migrated Skills historically used both ``ok`` and ``success``.
        # Accept both so a failed legacy result cannot be reported as success.
        ok = value.get("ok", value.get("success", True))
        return SkillResult(
            ok=bool(ok),
            code=value.get("code", "SUCCESS" if ok else "FAILED"),
            message=value.get("message", ""),
            data=value,
            run_id=run_id,
        )

    return SkillResult(
        ok=True,
        code="SUCCESS",
        message="Skill execution succeeded",
        data=value,
        run_id=run_id,
    )


class RobotContext:
    """Stable dependency surface shared by pipelines and Skills.

    The first migration step intentionally delegates to the owning Skill.  It
    lets us move task code to explicit dependencies without opening duplicate
    sockets or breaking existing Skill helpers.  Hardware construction still
    remains lazy in ``skills.base.Skill``.
    """

    def __init__(self, skill):
        self.skill = skill

    @property
    def config(self):
        return self.skill.config

    @property
    def save_path(self):
        return self.skill.save_path

    def arm(self, side="left"):
        return self.skill.arm_for(side)

    def gripper(self, side="left"):
        return self.skill.gripper_for(side)

    def camera(self, side="left"):
        return self.skill.get_camera(side)

    def twin(self, side="left"):
        return self.skill.twin_for(side)

    def __getattr__(self, name):
        """Compatibility bridge while individual helpers are migrated."""
        return getattr(self.skill, name)


def new_run_id():
    return "%s-%s" % (
        datetime.now().strftime("%Y%m%d%H%M%S"),
        uuid.uuid4().hex[:8],
    )


def _json_safe(value):
    """Keep CLI result serialization tolerant of numpy-like payloads."""
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "tolist"):
        try:
            return value.tolist()
        except Exception:
            pass
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)
