#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Atomic visual grasp skill using the shared grasp pipeline."""

from termcolor import cprint

from skills.base import Skill, register_skill


@register_skill("grasp")
class GraspSkill(Skill):
    """Run RGB-D → AnyGrasp → Twin → gripper grasping for either arm."""

    def execute(self, **kwargs):
        object_name = kwargs.get("object")
        if not object_name:
            cprint("[grasp] Missing required kwarg: object", "red")
            return False
        side = kwargs.get("side", "left")
        location = kwargs.get("location", "desk_front")
        hold_after_grasp = bool(kwargs.get("hold_after_grasp", False))
        observation_pose = kwargs.get("observation_pose")
        use_vlm_grounding = bool(kwargs.get("use_vlm_grounding", True))
        max_attempts = max(1, int(kwargs.get("max_attempts", 2)))
        if side not in ("left", "right"):
            cprint(f"[grasp] Unsupported side: {side}", "red")
            return False

        grasp_args = {
            "side": side,
            "location": location,
            "hold_after_grasp": hold_after_grasp,
            "observation_pose": observation_pose,
            "use_vlm_grounding": use_vlm_grounding,
        }
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                cprint(
                    f"[grasp] 视觉抓取失败，开始第 {attempt} 次尝试 / {max_attempts}",
                    "yellow",
                )
            # visual_grasp owns its recovery: after a failed candidate or
            # observation pose it returns the arm and opens the gripper
            # before this fresh perception attempt starts.
            if self.visual_grasp(object_name, **grasp_args):
                return True

        cprint(f"[grasp] 视觉抓取在 {max_attempts} 次尝试后仍失败", "red")
        return False
