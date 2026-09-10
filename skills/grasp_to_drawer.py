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

Drawer open/close replays pre-recorded SDK trajectories
(``recorded_trajectories/right/{open,close}_drawer.json``) — the
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
from termcolor import cprint

from skills.base import Skill, register_skill
from core.drawer_executor import DRAWER_TRAJECTORY_SPEED as DEFAULT_DRAWER_TRAJECTORY_SPEED


@register_skill("grasp_to_drawer")
class GraspToDrawer(Skill):
    """Left-arm grasp + right-arm drawer open/close with dual-arm handover.

    Composed of: base visual grasp + core dual-handover replay + drawer
    trajectory replay. No inheritance from PickAndPlaceSkill.
    """

    DRAWER_TRAJECTORY_SPEED = DEFAULT_DRAWER_TRAJECTORY_SPEED

    def execute(self, **kwargs):
        data = kwargs if kwargs.get("object") else self.json_parser.get_command()
        obj = data.get("object", "orange")
        # Detection mode pass-through: YOLO-direct (False) avoids VLM
        # misgrounding when the target is unambiguous at the observation
        # poses; VLM grounding (default True) stays opt-in for disambiguation.
        use_vlm = bool(data.get("use_vlm_grounding", True))

        cprint(f"[grasp_to_drawer] 目标物体: {obj} (use_vlm_grounding={use_vlm})", "cyan")

        # ── Phase 1: 左臂抓取 & 右臂同时打开抽屉（并行）──
        cprint("[grasp_to_drawer] 阶段1: 左臂抓取 & 右臂开抽屉（并行）", "yellow")
        drawer_result = {"ok": False}

        def _open_drawer_thread():
            try:
                drawer_result["ok"] = self.drawer_pipeline.open(
                    speed=self.DRAWER_TRAJECTORY_SPEED
                )
            except Exception as e:
                cprint(f"[grasp_to_drawer] 开抽屉异常: {e}", "red")

        t_drawer = threading.Thread(target=_open_drawer_thread)
        t_drawer.start()

        grasp_ok = self.visual_grasp(obj, side="left", use_vlm_grounding=use_vlm)
        t_drawer.join()

        if not grasp_ok:
            cprint("[grasp_to_drawer] 左臂抓取失败", "red")
            return False
        if not drawer_result["ok"]:
            cprint("[grasp_to_drawer] 右臂开抽屉失败", "red")
            return False

        # ── Phase 2: 两臂交接（左→右）──
        cprint("[grasp_to_drawer] 阶段2: 双臂交接（左→右）", "yellow")
        handover_ok = self.handover_pipeline.run(
            mode="dual",
            speed=1,
            require_confirmation=False,
            direction="left_to_right",
        )
        if not handover_ok:
            cprint("[grasp_to_drawer] 双臂交接失败", "red")
            return False

        # ── Phase 3: 右臂移动到 drawer_1_placement 并松开夹爪 ──
        cprint("[grasp_to_drawer] 阶段3: 右臂放置到抽屉", "yellow")
        if not self.drawer_pipeline.release_at(
                "drawer_1_placement", side="right"):
            cprint("[grasp_to_drawer] 放置到抽屉失败", "red")
            return False

        # ── Phase 5: 右臂关抽屉（轨迹回放，含回 home）──
        cprint("[grasp_to_drawer] 阶段5: 右臂关抽屉", "yellow")
        if not self.drawer_pipeline.close(
                speed=self.DRAWER_TRAJECTORY_SPEED):
            cprint("[grasp_to_drawer] 关抽屉失败", "red")
            return False

        cprint("[grasp_to_drawer] 完成", "green")
        return True
