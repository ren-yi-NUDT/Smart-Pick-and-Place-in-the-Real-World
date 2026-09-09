#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""右臂通过 pose_execute 命名位姿把物品递给用户。"""

import time

from skills.base import register_skill
from skills.pose_execute import PoseExecuteSkill


@register_skill("right_give_to_user")
class RightGiveToUserSkill(PoseExecuteSkill):
    """Move to the right-arm handover pose, release, then return home."""

    ARM_SIDE = "right"
    DEFAULT_POSE = "handover_pose"
    DEFAULT_SPEED = 15
    DEFAULT_RELEASE_WAIT = 2.0
    DEFAULT_HOME_SPEED = 30

    def execute(self, **kwargs):
        """Run the same named-pose and gripper path as ``pose_execute``."""
        pose_name = kwargs.get(
            "pose_name", kwargs.get("name", self.DEFAULT_POSE)
        )
        speed = float(kwargs.get("speed", self.DEFAULT_SPEED))
        release_wait = max(
            0.0, float(kwargs.get("release_wait", self.DEFAULT_RELEASE_WAIT))
        )
        home_speed = float(kwargs.get("home_speed", self.DEFAULT_HOME_SPEED))

        if not self.play_pose(
            pose_name, arm=self.ARM_SIDE, speed=speed, block=True
        ):
            return False

        if not self.play_hand_gesture("open", arm=self.ARM_SIDE):
            return False
        if release_wait:
            time.sleep(release_wait)
        return self.play_pose(
            "home", arm=self.ARM_SIDE, speed=home_speed, block=True
        )
