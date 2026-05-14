#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
World memory setup skill.

Usage:
    echo '{"object":"orange","container":"green bowl"}' | python run_skill.py world_memory_setup
"""

from skills.base import Skill, register_skill
from core.world_memory import setup_world_memory

@register_skill("world_memory_setup")
class WorldMemorySetupSkill(Skill):
    """
    Create a task-scoped world memory index for one pick/place task.

    This skill only initializes memory. It does not touch hardware.
    """

    def run(self, **kwargs):
        command = dict(kwargs)

        # Support fetch_from_user style input:
        #   {"container": "pink plate"}
        # If object is missing, mark it as unknown_from_user only when explicitly requested.
        # For now, keep missing object as None unless source=user_hand is provided.
        if command.get("source") == "user_hand" and not command.get("object"):
            command["object"] = "unknown_from_user"

        memory = setup_world_memory(command)

        return {
            "success": True,
            "skill": "world_memory_setup",
            "memory_id": memory.memory_id,
            "mode": memory.data["mode"],
            "status": memory.data["status"],
            "path": str(memory.path),
            "event_path": str(memory.event_path),
            "current_path": str(memory.current_path),
            "command": command,
            "required_workflow": [
                "index_task",
                "observe_or_detect",
                "critic_before_grasp",
                "run_grasp",
                "verify_grasp",
                "critic_before_destination",
                "run_destination_skill",
                "verify_destination",
                "verify_goal",
            ],
        }