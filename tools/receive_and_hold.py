#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从用户手中接收物品并保持在夹爪中（不放置）。

fetch_from_user 的"只接不放"变体：复用共享的
handover_pipeline.receive_from_user 流程（handover 位姿 → 开爪 → 等待 →
闭爪 → 力反馈校验 → 夹着回 home），成功后直接停止，物品留在夹爪里，
供后续技能（如 wipe_table）使用。

默认右臂：右臂在 robot_config.json 中只有 handover_pose/home 两个位姿，
_move_user_handover 会自动跳过缺失的过渡位姿；wipe_table 轨迹也是右臂。

用法:
    python3 tools/receive_and_hold.py
    echo '{"side":"right","wait_seconds":10}' | python3 tools/receive_and_hold.py
    echo '{"side":"right","speed":0.5}' | python3 tools/receive_and_hold.py
    SIM_MODE=1 python3 tools/receive_and_hold.py   # 仿真干跑
"""

import json
import os
import sys

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(project_root)
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, "dependence"))

import skills  # noqa: F401  # 触发技能注册
from skills.base import get_skill


def main():
    kwargs = {}
    if not sys.stdin.isatty():
        try:
            kwargs = json.load(sys.stdin)
        except json.JSONDecodeError:
            pass

    side = kwargs.get("side", "right")
    if side not in ("left", "right"):
        print("[receive_and_hold] unsupported side: %s" % side)
        sys.exit(2)

    skill = get_skill("receive_and_hold")(
        config_path="./robot_config.json", save_path="./log"
    )
    result = skill.run(**kwargs)
    sys.exit(0 if bool(result) else 1)


if __name__ == "__main__":
    main()
