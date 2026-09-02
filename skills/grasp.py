#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Atomic visual grasp skill using the shared grasp pipeline."""

from termcolor import cprint

from skills.base import Skill, register_skill


@register_skill("grasp")
class GraspSkill(Skill):
    """Run RGB-D → AnyGrasp → Twin → gripper grasping for either arm."""

    def run(self, **kwargs):
        object_name = kwargs.get("object")
        if not object_name:
            cprint("[grasp] Missing required kwarg: object", "red")
            return False
        side = kwargs.get("side", "left")
        location = kwargs.get("location", "desk_front")
        hold_after_grasp = bool(kwargs.get("hold_after_grasp", False))
        observation_pose = kwargs.get("observation_pose")
        use_vlm_grounding = bool(kwargs.get("use_vlm_grounding", True))
        if side not in ("left", "right"):
            cprint(f"[grasp] Unsupported side: {side}", "red")
            return False
        return self.visual_grasp(
            object_name,
            side=side,
            location=location,
            hold_after_grasp=hold_after_grasp,
            observation_pose=observation_pose,
            use_vlm_grounding=use_vlm_grounding,
        )
