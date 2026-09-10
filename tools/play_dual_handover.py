#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CLI shim — 回放引擎已迁至 core/dual_handover.py，保留原命令行入口。"""

import argparse


def main():
    from core.dual_handover import play

    parser = argparse.ArgumentParser(description="双臂交接位姿回放")
    parser.add_argument("--name", default=None, help="默认 dual_handover_timed_20260826_v2")
    parser.add_argument(
        "--speed", type=float, default=1,
        help="速度倍率，默认 1",
    )
    parser.add_argument(
        "--direction", choices=("left_to_right", "right_to_left"),
        default="left_to_right",
        help="交接方向（决定夹爪事件角色，航点两个方向共用）",
    )
    args = parser.parse_args()
    raise SystemExit(
        0 if play(args.name, args.speed, direction=args.direction) else 1
    )


if __name__ == "__main__":
    main()
