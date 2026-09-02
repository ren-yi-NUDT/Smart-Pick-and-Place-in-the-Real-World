#!/usr/bin/env python3
"""Print a compact, read-only project state summary for long robot sessions.

This command summarizes the workspace; it cannot force compaction of the
assistant's private conversation context.
"""
from __future__ import annotations

import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def run(command):
    try:
        result = subprocess.run(
            command,
            cwd=ROOT,
            text=True,
            capture_output=True,
            timeout=3,
            check=False,
        )
        return result.stdout.strip()
    except Exception as exc:  # noqa: BLE001
        return f"unavailable: {exc}"


def latest_plan():
    runs = sorted(
        (path for path in (ROOT / "log").glob("dual_vlm_sort_*") if path.is_dir()),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    for run_dir in runs:
        plan_path = run_dir / "plan.json"
        if not plan_path.exists():
            continue
        try:
            plan = json.loads(plan_path.read_text(encoding="utf-8"))
            return run_dir, plan
        except (OSError, json.JSONDecodeError):
            continue
    return None, None


def main():
    run_dir, plan = latest_plan()
    print("[compact] 项目状态摘要（只读）")
    print(f"root: {ROOT}")
    print("services:")
    ports = run(["ss", "-ltnp"])
    for port in (8030, 8031, 8032, 8033):
        rows = [line for line in ports.splitlines() if f":{port} " in line or f":{port}\t" in line]
        print(f"  {port}: {'up' if rows else 'down'}")
    if plan is None:
        print("latest_plan: none")
    else:
        actions = plan.get("actions", [])
        print(f"latest_plan: {run_dir / 'plan.json'}")
        print(
            "  mode={mode}, objects={objects}, actions={actions}".format(
                mode=plan.get("execution_mode", "unknown"),
                objects=plan.get("object_count", "?"),
                actions=len(actions),
            )
        )
        print(f"  queues={plan.get('pipeline_arm_queues', {})}")
    status = run([
        "git",
        "status",
        "--short",
        "--",
        "tools/dual_vlm_sorting.py",
        "skills/base.py",
        "dependence/twin_inference/sim_server.py",
        "robot_config.json",
        "configs/dual_vlm_sorting.json",
    ])
    print("tracked_changes:")
    print(status or "  clean")


if __name__ == "__main__":
    main()
