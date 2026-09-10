#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Reusable handover operations for user and dual-arm transfers."""

import time

from termcolor import cprint

class HandoverPipeline:
    """Own handover state transitions; callers only select a strategy."""

    def __init__(self, context):
        self.context = context
        self._owner = getattr(context, "skill", context)

    def receive_from_user(self, side="left", wait_seconds=4.0,
                          retry_wait_seconds=3.0, speed=15,
                          speed_scale=1.0):
        """Open the requested gripper, wait for a user, close and verify."""
        if side not in ("left", "right"):
            cprint("[handover] unsupported side: %s" % side, "red")
            return False
        try:
            speed_scale = float(speed_scale)
        except (TypeError, ValueError):
            cprint("[handover] invalid speed_scale: %s" % speed_scale, "red")
            return False
        if speed_scale <= 0:
            cprint("[handover] speed_scale must be positive", "red")
            return False

        motion_speed = self._scaled_speed(speed, speed_scale)
        return_speed = self._scaled_speed(30, speed_scale)
        if not self._move_user_handover(side, speed=motion_speed):
            return False
        if not self._owner.control_hand(cmd_type="open", side=side):
            return False
        cprint("[handover] waiting for user to place the object", "cyan")
        time.sleep(max(0.0, float(wait_seconds)))

        if not self._owner.control_hand(cmd_type="close", side=side):
            return False
        time.sleep(0.5)
        if not self._owner.check_grasping_object(side=side):
            cprint("[handover] no object detected; retrying once", "yellow")
            if not self._owner.control_hand(cmd_type="open", side=side):
                return False
            time.sleep(max(0.0, float(retry_wait_seconds)))
            if not self._owner.control_hand(cmd_type="close", side=side):
                return False
            time.sleep(0.5)
            if not self._owner.check_grasping_object(side=side):
                self._return_home(side, speed=return_speed)
                return False

        return self._return_home(side, speed=return_speed)

    def release_to_user(self, side="left", speed=15, release_wait=2.0):
        """Move to the arm's named handover pose, release, then go home."""
        if side not in ("left", "right"):
            cprint("[handover] unsupported arm side: %s" % side, "red")
            return False
        if self.context.config.get_pose("handover_pose", side=side) is None:
            cprint("[handover] missing handover_pose", "red")
            return False
        try:
            release_wait = max(0.0, float(release_wait))
        except (TypeError, ValueError):
            cprint("[handover] invalid release_wait: %s" % release_wait, "red")
            return False

        if not self._move_user_handover(side, speed=speed):
            return False
        if not self._owner.control_hand(cmd_type="open", side=side):
            return False
        time.sleep(release_wait)
        return self._return_home(side, speed=30)

    def dual(self, direction="left_to_right", name=None, speed=1,
             require_confirmation=False):
        """Replay the validated dual-arm handover using shared clients."""
        if direction not in ("left_to_right", "right_to_left"):
            cprint("[handover] unsupported direction: %s" % direction, "red")
            return False
        from core.dual_handover import play
        try:
            return bool(play(
                name=name,
                speed=speed,
                direction=direction,
                require_confirmation=require_confirmation,
                skill=self._owner,
            ))
        except Exception as exc:
            cprint("[handover] dual transfer failed: %s" % exc, "red")
            return False

    def run(self, mode="user_release", **kwargs):
        """Common dispatch used by the public handover Skills."""
        if mode == "receive_user":
            return self.receive_from_user(**kwargs)
        if mode == "user_release":
            return self.release_to_user(**kwargs)
        if mode == "dual":
            return self.dual(**kwargs)
        raise ValueError("unknown handover mode: %s" % mode)

    def _move_user_handover(self, side, speed=15):
        if self.context.config.get_pose("handover_pose", side=side) is None:
            cprint("[handover] missing handover_pose for %s arm" % side, "red")
            return False
        return bool(self._owner.control_arm(
            pose_type="handover_pose", speed=speed, side=side
        ))

    def _return_home(self, side, speed=30):
        return bool(self._owner.control_arm(
            pose_type="home", speed=speed, side=side
        ))

    @staticmethod
    def _scaled_speed(speed, scale):
        """Scale a controller speed while keeping it in the SDK range."""
        return max(1, min(100, int(round(float(speed) * float(scale)))))
