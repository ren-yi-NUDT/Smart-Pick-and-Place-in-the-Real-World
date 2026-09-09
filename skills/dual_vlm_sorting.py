#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Registered Skill facade for the dual-arm VLM sorting orchestrator."""

import os

from skills.base import Skill, register_skill


@register_skill("dual_vlm_sorting")
class DualVlmSortingSkill(Skill):
    """Expose the existing sorter through the common Skill lifecycle."""

    def execute(self, **kwargs):
        if kwargs.get("sim"):
            os.environ["SIM_MODE"] = "1"
        if kwargs.get("real_confirm"):
            os.environ["DUAL_SORT_REAL_CONFIRM"] = "1"

        from tools.dual_vlm_sorting import DualVlmSorter

        default_task_config = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "configs", "dual_vlm_sorting_real_fruit_vegetable.json",
        )

        sorter = DualVlmSorter(
            self.config_path,
            kwargs.get("task_config") or default_task_config,
            kwargs.get("log_root", self.save_path),
        )
        return sorter.run(
            execute=bool(kwargs.get("execute", False)),
            yes=bool(kwargs.get("yes", False)),
            move_to_observation=bool(kwargs.get("move_to_observation", True)),
        )
