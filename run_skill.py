#!/usr/bin/env python3
"""Unified entry point for robot skills.

Skills that return a structured dict (e.g. pick_and_place with full critic
pipeline) manage their own world memory internally. Atomic skills that return
True/False get automatic lightweight memory + critic via this runner.

Usage:
    echo '{"object":"orange","container":"bowl"}' | python3 run_skill.py pick_and_place
    echo '{"container":"pink plate"}' | python3 run_skill.py fetch_from_user
    python3 run_skill.py look_around
    python3 run_skill.py list
"""
import sys
import json
import argparse
import os


def main():
    project_root = os.path.dirname(os.path.abspath(__file__))
    os.chdir(project_root)
    sys.path.insert(0, project_root)
    sys.path.insert(0, os.path.join(project_root, "dependence"))

    parser = argparse.ArgumentParser(description="Robot Skill Runner")
    parser.add_argument("skill", help="Skill name to run (or 'list' to see all)")
    parser.add_argument("--config", default="./robot_config.json", help="Robot config path")
    parser.add_argument("--save-path", default="./log", help="Save directory")
    args = parser.parse_args()

    # Import all skills to trigger registration
    import skills
    from skills.base import get_skill, list_skills

    if args.skill == "list":
        print("Available skills:")
        for name in sorted(list_skills()):
            print(f"  - {name}")
        return

    skill_cls = get_skill(args.skill)
    if skill_cls is None:
        print(f"Unknown skill: {args.skill}")
        print(f"Available: {', '.join(sorted(list_skills()))}")
        sys.exit(1)

    skill = skill_cls(config_path=args.config, save_path=args.save_path)

    # Read JSON from stdin if available
    kwargs = {}
    if not sys.stdin.isatty():
        try:
            kwargs = json.load(sys.stdin)
        except json.JSONDecodeError:
            pass

    # Skills that return a structured dict manage their own world memory + critic
    self_managed = ("pick_and_place", "pick_and_place_v2", "world_memory_setup")

    if args.skill in self_managed:
        _run_self_managed(skill, args.skill, kwargs)
    else:
        _run_with_wrapper(skill, args.skill, kwargs)


def _run_self_managed(skill, skill_name, kwargs):
    """Run a self-managed skill (handles memory + critic internally)."""
    try:
        result = skill.run(**kwargs)

        if result is False:
            print("Skill execution failed")
            sys.exit(1)
        elif result is True:
            print("Skill execution succeeded")
        elif isinstance(result, dict):
            print(json.dumps(result, ensure_ascii=False, indent=2))
        elif result is not None:
            print(result)

    except Exception as e:
        print(f"Skill execution error: {type(e).__name__}: {e}")
        sys.exit(1)


def _run_with_wrapper(skill, skill_name, kwargs):
    """Run an atomic skill with automatic world memory + lightweight critic."""
    from core.world_memory import setup_world_memory

    command = dict(kwargs)
    command.setdefault("skill", skill_name)
    memory = setup_world_memory(command)

    memory.record_action({
        "type": f"{skill_name}_start",
        "input": kwargs,
    })

    try:
        result = skill.run(**kwargs)

        success = result is not False
        memory.record_action({
            "type": f"{skill_name}_result",
            "result": {"success": success, "raw": str(result)},
        })

        # Lightweight critic for atomic skills
        _run_lightweight_critic(memory, skill_name, kwargs, result)

        if result is False:
            print("Skill execution failed")
            sys.exit(1)
        elif result is True:
            print("Skill execution succeeded")
        elif isinstance(result, dict):
            print(json.dumps(result, ensure_ascii=False, indent=2))
        elif result is not None:
            print(result)

    except Exception as e:
        memory.record_action({
            "type": f"{skill_name}_error",
            "result": {"success": False, "error_type": type(e).__name__, "error": str(e)},
        })
        _run_lightweight_critic(memory, skill_name, kwargs, None, error=e)
        print(f"Skill execution error: {type(e).__name__}: {e}")
        sys.exit(1)


def _run_lightweight_critic(memory, skill_name, kwargs, result, error=None):
    """Run a lightweight goal-verification for atomic skills (advisory only)."""
    try:
        from core.world_model_critic import WorldModelCritic

        critic = WorldModelCritic(memory)
        success = result is not False and error is None
        obj = kwargs.get("object", kwargs.get("obj"))
        container = kwargs.get("container")

        # Map container to mode
        if container and container.lower() == "person":
            mode = "handover"
        elif container and container.lower() in ("trash", "垃圾桶", "garbage", "bin"):
            mode = "trash"
        elif container and container.lower() in ("desk", "桌子", "table"):
            mode = "desk_place"
        else:
            mode = container or "unknown"

        payload = {
            "success": success,
            "object": obj,
            "container": container,
            "mode": mode,
            "grasp_result": {"success": True},
            "destination_result": {"success": success},
            "resolved_failures": 0,
        }
        if error:
            payload["error"] = f"{type(error).__name__}: {error}"

        goal = critic.verify_goal(payload)
        memory.record_action({"type": f"{skill_name}_critic", "critic": goal})

        verdict = goal.get("success", False)
        if verdict:
            print(f"[critic] ✅ Goal verified")
        else:
            issues = [k for k, v in goal.get("conditions", {}).items() if not v]
            print(f"[critic] ⚠️  Goal check: {len(issues)} issue(s): {', '.join(issues)}")

    except Exception as e:
        print(f"[critic] ⚠️  Skipped: {e}")


if __name__ == "__main__":
    main()
