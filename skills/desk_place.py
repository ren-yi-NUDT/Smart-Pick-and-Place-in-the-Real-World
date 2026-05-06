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
import time
import numpy as np
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

    def execute_desk_placement_js(self):
        """Move to a random desk pose and release the object."""
        selected_pose_key = random.choice(self.DESK_POSE_KEYS)
        desk_pose = self.config.get_pose(selected_pose_key)

        if desk_pose is None:
            cprint(
                f"Error: {selected_pose_key} not defined in robot_config.json",
                "red",
            )
            return False

        cprint(
            f"=============== Moving to desk pose ({selected_pose_key}) =============",
            "cyan",
        )

        joint_angles = [
            desk_pose["J1"], desk_pose["J2"], desk_pose["J3"],
            desk_pose["J4"], desk_pose["J5"], desk_pose["J6"],
            desk_pose["J7"],
        ]

        trajectory = np.array([joint_angles])
        self.control_arm(trajectory=trajectory, speed=15)

        cprint(
            f"=============== Reached desk pose ({selected_pose_key}) =============",
            "green",
        )
        time.sleep(0.5)

        # Open hand to place on desk
        self.control_hand(cmd_type="open")
        cprint(
            "=============== Opened hand to place on desk =============",
            "green",
        )
        time.sleep(1)

        # Return to safe position
        self.control_arm(pose_type="grasp1", speed=30)
        cprint("=============== Returned to safe pose =============", "cyan")

        return True

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def run(self, **kwargs):
        """
        Execute the desk placement skill.

        Returns:
            bool: True if successful, False otherwise.
        """
        check = self.execute_desk_placement_js()
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
