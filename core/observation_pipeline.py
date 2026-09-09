#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared observation and VLM-analysis operations."""

import os
from datetime import datetime

from PIL import Image
from termcolor import cprint


class ObservationPipeline:
    """Capture configured views without embedding observation logic in Skills."""

    def __init__(self, context):
        self.context = context
        self._owner = getattr(context, "skill", context)

    def capture_at_handover(self, reset_pose="grasp1"):
        pose_name = "look_over_what_in_user_hand_pose"
        if self.context.config.get_pose(pose_name, side="left") is None:
            cprint("[observe] missing %s" % pose_name, "red")
            return None
        if not self._owner.control_arm(
                pose_type=pose_name, speed=15, side="left"):
            return None
        rgb, depth = self._owner.get_camera_obs(side="left")
        self._save_rgbd("handover", rgb, depth)
        analysis = self._owner.vlm.analyze(
            rgb,
            prompt="画面中有一只手，告诉我手里拿的是什么。请简洁回答，只说物品名称。",
        )
        if reset_pose:
            self._owner.control_arm(
                pose_type=reset_pose, speed=30, side="left"
            )
        return analysis

    def scan_workspace(self, reset_pose="grasp1"):
        images = {}
        poses = self.context.config.get_arm_config("left").get(
            "default_traj_js", {}
        )
        for name in poses:
            if "grasp" not in name:
                continue
            if not self._owner.control_arm(
                    pose_type=name, speed=30, side="left"):
                continue
            rgb, depth = self._owner.get_camera_obs(side="left")
            paths = self._save_rgbd("look_around", rgb, depth, suffix=name)
            images[name] = {"rgb": rgb, "depth": depth, **paths}

        if not images:
            cprint("[observe] no workspace observation poses found", "red")
            return None
        first_rgb = next(iter(images.values()))["rgb"]
        analysis = self._owner.vlm.analyze(
            first_rgb,
            prompt=(
                "请分析图片中的物品及其空间关系，按以下格式回答：\n\n"
                "【物品列表】\n"
                "1. 物品名称 - 位于图片的(左上/右上/左下/右下/中间)位置\n\n"
                "【空间关系】\n"
                "是否有物品被某些容器装着？如果有，请列出。"
            ),
        )
        if reset_pose:
            self._owner.control_arm(
                pose_type=reset_pose, speed=30, side="left"
            )
        return analysis

    def inspect_drawer(self, reset_pose="home"):
        pose_name = "drawer_1_placement"
        if self.context.config.get_pose(pose_name, side="right") is None:
            cprint("[observe] missing right-arm %s" % pose_name, "red")
            return None
        if not self._owner.control_arm(
                pose_type=pose_name, speed=15, side="right"):
            return None
        rgb, depth = self._owner.get_camera_obs(side="right")
        self._save_rgbd("look_around_drawer", rgb, depth)
        analysis = self._owner.vlm.analyze(
            rgb,
            prompt=(
                "这是一张抽屉内部的照片，请你描述在抽屉里都看到了什么，"
                "只输出词语即可，忽视抽屉里的泡沫板"
            ),
        )
        if reset_pose:
            self._owner.control_arm(
                pose_type=reset_pose, speed=30, side="right"
            )
        return analysis

    def _save_rgbd(self, prefix, rgb, depth, suffix=""):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        suffix = ("_" + str(suffix)) if suffix else ""
        rgb_path = os.path.join(
            self.context.save_path, "%s_rgb_%s%s.png" % (prefix, stamp, suffix)
        )
        depth_path = os.path.join(
            self.context.save_path, "%s_depth_%s%s.png" % (prefix, stamp, suffix)
        )
        Image.fromarray(rgb).save(rgb_path)
        Image.fromarray(depth).save(depth_path)
        cprint("[observe] saved: %s" % rgb_path, "cyan")
        return {"rgb_path": rgb_path, "depth_path": depth_path}
