#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Receive an object from a user and place it through shared pipelines.

该 Skill 的方向是用户交给机械臂；它不负责把物品递回用户。
"""

import random

from termcolor import cprint

from skills.base import Skill, register_skill


@register_skill("fetch_from_user")
class FetchFromUserSkill(Skill):
    """Composite Skill: user handover receive + destination placement."""

    def execute(self, **kwargs):
        data = kwargs if kwargs.get("container") else self.json_parser.get_command()
        if data is None:
            cprint("No valid JSON input received", "red")
            return False

        container = data.get("container")
        if not container:
            cprint("JSON input missing required field: container", "red")
            return False
        side = data.get("side", "left")
        if side not in ("left", "right"):
            cprint("Unsupported arm side: %s" % side, "red")
            return False
        speed_scale = data.get("speed_scale", data.get("speed", 1.0))

        cprint("[fetch_from_user] receiving object with %s arm" % side, "cyan")
        if not self.handover_pipeline.run(
                mode="receive_user", side=side,
                wait_seconds=data.get("wait_seconds", 4.0),
                retry_wait_seconds=data.get("retry_wait_seconds", 3.0),
                speed_scale=speed_scale):
            cprint("[fetch_from_user] receive failed", "red")
            return False

        normalized = str(container).lower()
        if normalized in ("trash", "垃圾桶", "garbage", "bin"):
            return self.place_pipeline.place_at_named_pose(
                "throw_to_trash_pose", side=side
            )

        if normalized in ("desk", "桌子", "table"):
            pose = data.get("pose") or random.choice(
                ("desk_pose_1", "desk_pose_2", "desk_pose_3")
            )
            return self.place_pipeline.place_at_named_pose(
                pose, side=side
            )

        # All visual destinations use the same RGB-D/VLM/YOLO/Twin placement
        # chain. No old private placement helpers are called here.
        return self.place_pipeline.run(
            container=container,
            side=side,
            object_name=data.get("object") or data.get("object_name"),
            object_size_m=data.get("object_size_m"),
            use_vlm_grounding=data.get("use_vlm_grounding", True),
        )
