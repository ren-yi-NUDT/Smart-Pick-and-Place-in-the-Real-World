#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Desk placement skill -- extracted from Planner.execute_desk_placement_js.

Randomly selects one of the desk poses, moves there, opens the hand to
place the object, then returns to a safe position.

Usage (CLI):
    echo '{}' | python3 run_skill.py desk_place
"""

import random
from termcolor import cprint

from skills.base import Skill, register_skill


@register_skill("desk_place")
class DeskPlaceSkill(Skill):
    """
    Atomic desk-placement skill.

    Randomly selects one of desk_pose_1/2/3, moves there, opens the hand,
    and returns to a safe position.
    """

    DESK_POSE_KEYS = ["desk_pose_1", "desk_pose_2", "desk_pose_3"]

    def __init__(self, **kw):
        super().__init__(**kw)

    def execute_desk_placement_js(self, pose_name=None):
        """Compatibility wrapper around the shared fixed-place pipeline."""
        selected_pose_key = pose_name or random.choice(self.DESK_POSE_KEYS)
        cprint(
            f"=============== Moving to desk pose ({selected_pose_key}) =============",
            "cyan",
        )
        return self.place_pipeline.place_at_named_pose(
            selected_pose_key, side="left"
        )

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def execute(self, **kwargs):
        """
        Execute the desk placement skill.

        Returns:
            bool: True if successful, False otherwise.
        """
        check = self.execute_desk_placement_js(kwargs.get("pose"))
        if check:
            cprint(
                "D=================== Successfully completed the desk placement task ===================",
                "green",
            )
        else:
            cprint(
                "D=================== Desk placement task failed ===================",
                "red",
            )
        return check
