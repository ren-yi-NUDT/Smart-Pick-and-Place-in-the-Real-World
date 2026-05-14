#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task-scoped world memory for Smart Pick-and-Place.

This module creates and manages a lightweight JSON world-memory index
for each robot task. It does not connect to hardware.
"""

import json
import time
import uuid
from pathlib import Path

SPECIAL_CONTAINER_MODES = {
    "person": "handover",
    "trash": "trash",
    "garbage": "trash",
    "bin": "trash",
    "垃圾桶": "trash",
    "desk": "desk_place",
    "table": "desk_place",
    "桌子": "desk_place",
}

def _safe_name(value):
    """
    Convert an arbitrary object/container string into a safe filename segment.
    """
    if value is None:
        return "unknown"

    value = str(value).strip()
    if not value:
        return "unknown"

    for ch in [" ", ",", "/", "\\", ":", ";", "|", "\t", "\n"]:
        value = value.replace(ch, "_")

    # Avoid extremely long filenames
    return value[:80]

def infer_task_mode(container):
    """
    Infer task mode from container string.

    Modes:
    - normal_place: place object into/onto a detected container
    - handover: hand object to a person
    - trash: throw object into trash/bin
    - desk_place: place object on desk/table
    """
    if container is None:
        return "normal_place"

    key = str(container).strip().lower()
    return SPECIAL_CONTAINER_MODES.get(key, "normal_place")

class WorldMemory:
    """
    A task-scoped world memory index.

    It stores:
    - command
    - task goal
    - inferred mode
    - robot state
    - object/container indexes
    - spatial relations
    - observations
    - actions
    - critic records
    - verification records
    - failures
    """

    def __init__(self, memory_id, command, root="memory/world"):
        self.memory_id = memory_id
        self.command = command or {}
        self.root = Path(root)

        self.task_dir = self.root / "tasks"
        self.event_dir = self.root / "events"

        self.task_dir.mkdir(parents=True, exist_ok=True)
        self.event_dir.mkdir(parents=True, exist_ok=True)

        self.data = {
            "memory_id": self.memory_id,
            "created_at": time.time(),
            "updated_at": time.time(),

            "command": self.command,
            "mode": infer_task_mode(self.command.get("container")),

            "task_goal": {
                "object": self.command.get("object"),
                "container": self.command.get("container"),
                "source": self.command.get("source", "workspace"),
            },

            "robot_state": {
                "holding": None,
                "last_action": None,
            },

            "object_index": {},
            "container_index": {},
            "spatial_relations": [],

            "observation_history": [],
            "action_history": [],
            "critic_records": [],
            "verification_records": [],
            "failure_records": [],

            "status": "initialized",
        }

    @property
    def path(self):
        return self.task_dir / f"{self.memory_id}.json"

    @property
    def event_path(self):
        return self.event_dir / f"{self.memory_id}.jsonl"

    @property
    def current_path(self):
        return self.root / "current.json"

    def save(self):
        """
        Save full memory index and update current pointer.
        """
        self.data["updated_at"] = time.time()

        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(self.data, f, ensure_ascii=False, indent=2)

        current = {
            "memory_id": self.memory_id,
            "path": str(self.path),
            "event_path": str(self.event_path),
            "status": self.data.get("status"),
            "mode": self.data.get("mode"),
            "updated_at": self.data["updated_at"],
        }

        with open(self.current_path, "w", encoding="utf-8") as f:
            json.dump(current, f, ensure_ascii=False, indent=2)

    def event(self, event_type, payload):
        """
        Append one event to jsonl event log.
        """
        item = {
            "time": time.time(),
            "type": event_type,
            "payload": payload,
        }

        with open(self.event_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(item, ensure_ascii=False) + "\n")

    def set_status(self, status):
        self.data["status"] = status
        self.event("status", {"status": status})
        self.save()

    def add_observation(self, observation):
        observation = observation or {}
        observation.setdefault("time", time.time())

        self.data["observation_history"].append(observation)
        self.event("observation", observation)

        for item in observation.get("objects", []):
            item_id = item.get("id")
            if not item_id:
                continue

            role = item.get("role", "object")
            item["last_seen"] = observation["time"]

            if role == "container":
                self.data["container_index"][item_id] = {
                    **self.data["container_index"].get(item_id, {}),
                    **item,
                }
            else:
                self.data["object_index"][item_id] = {
                    **self.data["object_index"].get(item_id, {}),
                    **item,
                }

        for rel in observation.get("relations", []):
            rel.setdefault("active", True)
            rel.setdefault("time", observation["time"])
            self.data["spatial_relations"].append(rel)

        if "robot_state" in observation:
            self.data["robot_state"].update(observation["robot_state"])

        self.save()

    def record_action(self, action):
        action = action or {}
        action.setdefault("time", time.time())

        self.data["action_history"].append(action)
        self.data["robot_state"]["last_action"] = action

        self.event("action", action)
        self.save()

    def record_critic(self, record):
        record = record or {}
        record.setdefault("time", time.time())

        self.data["critic_records"].append(record)
        self.event("critic", record)
        self.save()

    def record_verification(self, record):
        record = record or {}
        record.setdefault("time", time.time())

        self.data["verification_records"].append(record)
        self.event("verification", record)
        self.save()

    def record_failure(self, failure):
        failure = failure or {}
        failure.setdefault("time", time.time())

        self.data["failure_records"].append(failure)
        self.data["status"] = "blocked_or_failed"

        self.event("failure", failure)
        self.save()

    def to_summary(self):
        """
        Return a compact summary suitable for CLI output.
        """
        return {
            "memory_id": self.memory_id,
            "mode": self.data.get("mode"),
            "status": self.data.get("status"),
            "path": str(self.path),
            "event_path": str(self.event_path),
            "current_path": str(self.current_path),
        }

def setup_world_memory(command, root="memory/world"):
    """
    Create a new task-scoped world memory index.
    """
    command = command or {}

    ts = time.strftime("%Y%m%d_%H%M%S")
    short_uuid = uuid.uuid4().hex[:6]

    obj = _safe_name(command.get("object", "unknown_object"))
    container = _safe_name(command.get("container", "unknown_container"))

    memory_id = f"task_{ts}_{obj}_to_{container}_{short_uuid}"

    memory = WorldMemory(
        memory_id=memory_id,
        command=command,
        root=root,
    )

    memory.record_action({
        "type": "memory_setup",
        "command": command,
    })

    memory.set_status("ready")

    return memory