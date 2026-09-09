#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Reusable drawer operations.

The recorded trajectory remains the low-level drawer actuator. This pipeline
owns its lifecycle and the right-arm release transition so composite Skills
do not duplicate hardware calls.
"""

import time

from termcolor import cprint
from core.drawer_executor import DRAWER_TRAJECTORY_SPEED


class DrawerPipeline:
    """Canonical drawer open/close pipeline; default trajectory speed is 0.5x."""

    def __init__(self, context):
        self.context = context
        self._owner = getattr(context, "skill", context)
        self._executor = None

    def open(self, speed=DRAWER_TRAJECTORY_SPEED):
        return self._play("open", speed)

    def close(self, speed=DRAWER_TRAJECTORY_SPEED):
        return self._play("close", speed)

    def release_at(self, pose_name="drawer_1_placement", side="right",
                   approach_speed=15, retreat_speed=30,
                   release_wait=1.0):
        """Move to a fixed receptacle pose, release and retreat safely."""
        pose = self.context.config.get_pose(pose_name, side=side)
        if pose is None:
            cprint("[drawer] missing pose: %s" % pose_name, "red")
            return False
        if not self._owner.control_arm(
                pose_type=pose_name, speed=approach_speed, side=side):
            return False
        if not self._owner.control_hand(cmd_type="open", side=side):
            return False
        time.sleep(max(0.0, float(release_wait)))
        return bool(self._owner.control_arm(
            pose_type="home", speed=retreat_speed, side=side
        ))

    def run(self, action, **kwargs):
        if action == "open":
            return self.open(**kwargs)
        if action == "close":
            return self.close(**kwargs)
        if action == "release":
            return self.release_at(**kwargs)
        raise ValueError("unknown drawer action: %s" % action)

    def _play(self, action, speed):
        if self._executor is None:
            from core.drawer_executor import create_drawer_executor
            drawer_cfg = self.context.config.shared.get("drawer", {})
            drawer_side = drawer_cfg.get("arm", "right")
            # Reuse the owner's clients.  Creating a second persistent arm
            # socket here can be queued behind the handover client's socket
            # by the legacy bridge, which accepts clients serially.
            self._executor = create_drawer_executor(
                self.context.config,
                arm_client=self._owner.arm_for(drawer_side),
                gripper_client=self._owner.gripper_for(drawer_side),
            )
        try:
            name = "open_drawer" if action == "open" else "close_drawer"
            return bool(self._executor.play(name, speed=speed))
        except Exception as exc:
            cprint("[drawer] %s trajectory failed: %s" % (action, exc), "red")
            return False
