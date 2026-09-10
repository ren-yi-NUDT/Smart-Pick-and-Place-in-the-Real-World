#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dual-arm handover skill — replay a recorded timed left↔right handover.

The arm waypoints are shared by both directions; ``direction`` only swaps
which gripper closes (receiver) and which opens (giver).

This Skill transfers an object between the two robot arms. It does not hand
an object directly to the user; use ``right_give_to_user`` for that action.
"""

from termcolor import cprint

from skills.base import Skill, register_skill


@register_skill("dual_handover")
class DualHandoverSkill(Skill):
    """Run a recorded dual-arm handover trajectory in either direction."""

    def execute(self, **kwargs):
        name = kwargs.get("name")
        speed = kwargs.get("speed", 1)
        direction = kwargs.get("direction", "left_to_right")
        if direction not in ("left_to_right", "right_to_left"):
            cprint(f"[dual_handover] 未知方向: {direction}", "red")
            return False

        cprint(
            f"[dual_handover] {direction} ({name or 'default'}, speed={speed})",
            "cyan",
        )
        try:
            return self.handover_pipeline.run(
                mode="dual",
                name=name,
                speed=speed,
                direction=direction,
                require_confirmation=bool(kwargs.get("require_confirmation", False)),
            )
        except Exception as exc:
            cprint(f"[dual_handover] 回放失败: {exc}", "red")
            return False
