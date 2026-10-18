#!/usr/bin/env python3
"""Unified entry point for robot skills.

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


def _ensure_anygrasp_environment(skill):
    """Re-exec skills in the environment containing the AnyGrasp dependencies."""
    if skill == "list" or os.environ.get("ANYGRASP_ENV_READY") == "1":
        return
    target = os.environ.get(
        "ANYGRASP_PYTHON", "/home/zz/anaconda3/envs/anygrasp/bin/python"
    )
    if not os.path.isfile(target) or os.path.realpath(sys.executable) == os.path.realpath(target):
        return
    env = os.environ.copy()
    env["ANYGRASP_ENV_READY"] = "1"
    cudnn = "/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib"
    env["LD_LIBRARY_PATH"] = ":".join(
        part for part in (cudnn, env.get("LD_LIBRARY_PATH", "")) if part
    )
    os.execvpe(target, [target, *sys.argv], env)


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
    _ensure_anygrasp_environment(args.skill)

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

    result = skill.run(**kwargs)
    if result is False:
        print("Skill execution failed")
        sys.exit(1)
    elif result is True:
        print("Skill execution succeeded")
    elif isinstance(result, dict):
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
