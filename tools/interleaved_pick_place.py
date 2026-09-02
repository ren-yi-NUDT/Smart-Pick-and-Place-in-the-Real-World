#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interleaved pick/pick, place/place orchestrator (planner layer).

双臂流水线编排 —— 只负责"什么时候调用什么"；执行、轨迹校验、
抓取确认与恢复全部由 run_skill.py 的原子技能（grasp / place）负责。

设计要点：
- 每步以子进程调用 run_skill.py：技能进程退出即释放其 TCP 客户端，
  不会挤占单客户端的 arm/gripper/twin 服务。
- 成败判定只用 run_skill.py 的退出码（0 成功 / 1 失败）。
- 任一步失败即停止后续步骤（现场人工接管）。
- 节奏：轮1抓（左臂甜椒 → 右臂苹果）→ 轮1放（左→粉碗 → 右→蓝盘）
  → 轮2（右臂桃子抓/放蓝盘）→ 轮3（右臂橘子抓/放蓝盘）。

用法（anygrasp 环境）：
    python tools/interleaved_pick_place.py            # 执行
    python tools/interleaved_pick_place.py --dry-run  # 只打印调用计划
"""

import argparse
import json
import os
import subprocess
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 编排计划：(skill, kwargs)
# use_vlm_grounding=True：VLM 先框目标再过滤 AnyGrasp 候选，
# 规避 YOLO 多框歧义（apple 双框 0.8/0.8、blue plate conf 0.2）。
#
# 变更记录（14:12）：首跑第 2 步右臂抓苹果失败（VLM 框正确但 AnyGrasp
# 在框内无抓取点，疑似橘子紧贴苹果导致点云连通）。调整：把苹果放到
# 最后，先抓走桃子/橘子再让苹果周边点云独立后重新感知。
#
# 变更记录（14:26）：CMLLR 手动放下桃子（Twin 对右臂持物→蓝盘连续
# 两次全拒：blue plate / plate 词都一样，规划层问题非关键词问题）。
# 当前现场：甜椒→粉碗✅，桃子→蓝盘✅（手动），剩橘子、苹果。
#
# 变更记录（16:2x）：切到仿真分拣（CMLLR 改了 place 逻辑：home 同步 +
# object_size_m 进 Twin）。场景 = sim_server --scene fruit：
#   in 平台(0.25,-0.35)：苹果(左格) + 橘子(右格)
#   白盘(0.25,-0.15) ← 右臂苹果；紫碗(-0.02,-0.60) ← 左臂橘子
# 通用词 plate/bowl（无颜色词）。真机结论：place_home_sync=True 默认。
PLAN = [
    # ---- 右臂：苹果 → 白盘 ----
    ("grasp", {"object": "apple", "side": "right",
               "use_vlm_grounding": True}),
    ("place", {"container": "plate", "side": "right",
               "use_vlm_grounding": True}),
    # ---- 左臂：橘子 → 紫碗 ----
    ("grasp", {"object": "orange", "side": "left",
               "use_vlm_grounding": True}),
    ("place", {"container": "bowl", "side": "left",
               "use_vlm_grounding": True}),
]


def call_skill(skill, kwargs):
    """Invoke one atomic skill through the project's unified entry point."""
    payload = json.dumps(kwargs, ensure_ascii=False)
    print(f"\n=== CALL {skill}: {payload}", flush=True)
    proc = subprocess.run(
        [sys.executable, "run_skill.py", skill],
        input=payload, text=True, cwd=PROJECT_ROOT,
    )
    ok = proc.returncode == 0
    print(f"=== {'OK ' if ok else 'FAIL'} {skill}: {payload}", flush=True)
    return ok


def main():
    parser = argparse.ArgumentParser(description="interleaved pick/place orchestrator")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the call plan without moving hardware")
    args = parser.parse_args()

    print("[orchestrator] 轮流抓、轮流放，共 %d 步：" % len(PLAN), flush=True)
    for i, (skill, kwargs) in enumerate(PLAN, 1):
        print(f"  {i:02d}. {skill}: {json.dumps(kwargs, ensure_ascii=False)}",
              flush=True)
    if args.dry_run:
        return

    for i, (skill, kwargs) in enumerate(PLAN, 1):
        if not call_skill(skill, kwargs):
            print(f"\n[orchestrator] 第 {i} 步失败，停止后续步骤；"
                  f"请现场确认机械臂与夹爪状态。", flush=True)
            sys.exit(1)

    print("\n[orchestrator] 全部完成：甜椒→粉碗，苹果/桃子/橘子→蓝盘", flush=True)


if __name__ == "__main__":
    main()
