#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Grasp-to-drawer skill — dual-arm coordination.

Pipeline:
  1. Left arm grasps the object  &  right arm opens the drawer  (parallel)
  2. Dual-arm handover (left → right)
  3. Right arm moves to `drawer_1_placement` and releases the gripper
  4. Right arm retreats to home (clears the drawer interior)
  5. Right arm closes the drawer (trajectory replay)

Drawer open/close is driven by ``pose_execute.play_open_drawer`` /
``play_close_drawer`` — these replay pre-recorded SDK trajectories
(``recorded_trajectories/right/{open,close}_drawer.json``) and are the
authoritative way to operate the drawer.

Usage (CLI):
    echo '{"object":"orange"}' | python3 run_skill.py grasp_to_drawer

    # or programmatically:
    from skills.base import get_skill
    Skill = get_skill("grasp_to_drawer")
    skill = Skill()
    skill.run(object="orange")
"""

import threading
import time
from termcolor import cprint

from skills.base import register_skill
from skills.pick_and_place import PickAndPlaceSkill


@register_skill("grasp_to_drawer")
class GraspToDrawer(PickAndPlaceSkill):
    """Left-arm grasp + right-arm drawer open/close with dual-arm handover."""

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def run(self, **kwargs):
        data = kwargs if kwargs.get("object") else self.json_parser.get_command()
        obj = data.get("object", "orange")

        cprint(f"[grasp_to_drawer] 目标物体: {obj}", "cyan")

        # ── Phase 1: 左臂抓取 & 右臂同时打开抽屉（并行）──
        cprint("[grasp_to_drawer] 阶段1: 左臂抓取 & 右臂开抽屉（并行）", "yellow")
        drawer_result = {"ok": False}

        def _open_drawer_thread():
            try:
                from skills.pose_execute import PoseExecuteSkill
                pe = PoseExecuteSkill(
                    config_path=self.config_path, save_path=self.save_path
                )
                drawer_result["ok"] = pe.play_open_drawer(speed=1.5)
            except Exception as e:
                cprint(f"[grasp_to_drawer] 开抽屉异常: {e}", "red")
                drawer_result["ok"] = False

        t_drawer = threading.Thread(target=_open_drawer_thread)
        t_drawer.start()

        # 左臂抓取（复用生产级流水线）
        grasp_ok = self._visual_grasp_phase(obj, side="left")
        t_drawer.join()

        if not grasp_ok:
            cprint("[grasp_to_drawer] 左臂抓取失败", "red")
            return False
        if not drawer_result["ok"]:
            cprint("[grasp_to_drawer] 右臂开抽屉失败", "red")
            return False

        # ── Phase 2: 两臂交接（左→右，复用验证过的 4 步序列）──
        cprint("[grasp_to_drawer] 阶段2: 双臂交接（左→右）", "yellow")
        if not self._delegate_to_left_arm(container="drawer"):
            cprint("[grasp_to_drawer] 双臂交接失败", "red")
            return False

        # ── Phase 3: 右臂移动到 drawer_1_placement 并松开夹爪 ──
        cprint("[grasp_to_drawer] 阶段3: 右臂放置到抽屉", "yellow")
        right_arm = self._ensure_right_arm()
        right_gripper = self._ensure_right_gripper()
        right_cfg = self.config.get_arm_config("right")
        place_pose = right_cfg.get("drawer_1_placement")
        if place_pose is None:
            cprint("[grasp_to_drawer] drawer_1_placement 位姿不存在", "red")
            return False

        right_arm.move_to_named_pose(place_pose, speed=15)
        right_gripper.open()
        time.sleep(1)

        # ── Phase 4: 右臂脱离抽屉回 home ──
        cprint("[grasp_to_drawer] 阶段4: 右臂脱离抽屉回 home", "yellow")
        right_arm.move_to_named_pose(right_cfg["home"], speed=30)

        # ── Phase 5: 右臂关抽屉（轨迹回放，含回 home）──
        cprint("[grasp_to_drawer] 阶段5: 右臂关抽屉", "yellow")
        from skills.pose_execute import PoseExecuteSkill
        pe = PoseExecuteSkill(
            config_path=self.config_path, save_path=self.save_path
        )
        if not pe.play_close_drawer(speed=1.5):
            cprint("[grasp_to_drawer] 关抽屉失败", "red")
            return False

        cprint("[grasp_to_drawer] 完成", "green")
        return True

    # ------------------------------------------------------------------
    # Right-arm client caching (avoid re-connecting across phases)
    # ------------------------------------------------------------------
    def _ensure_right_arm(self):
        return self.arm_for("right")

    def _ensure_right_gripper(self):
        return self.gripper_for("right")
