#!/usr/bin/env python3
"""最小运动验证：SDK 直连 → 读关节 → 低速去 home → 回读比对。

用法：PYTHONPATH=/home/zz/Desktop/RM_API2/Python python3 tools/verify_motion.py
"""
import json
import sys

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

ARM_IP, ARM_PORT = "192.168.1.18", 8080
SPEED = 10  # 关节速度百分比，低速验证


def read_joints(arm):
    tag, state = arm.rm_get_current_arm_state()
    if tag != 0:
        print(f"读取状态失败 tag={tag}")
        sys.exit(1)
    return state["joint"]


def main():
    with open("robot_config.json") as f:
        home = json.load(f)["default_traj_js"]["home"]
    target = [home[f"J{i}"] for i in range(1, 7)]

    arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
    handle = arm.rm_create_robot_arm(ARM_IP, ARM_PORT, level=3)
    if handle.id == -1:
        print(f"FAIL: 无法连接 {ARM_IP}:{ARM_PORT}")
        sys.exit(1)
    print(f"已连接 {ARM_IP}:{ARM_PORT}")

    before = read_joints(arm)
    print("运动前:", [f"{j:.2f}" for j in before])

    tag = arm.rm_movej(target, SPEED, 0, 0, 1)  # v=10 低速, block=1 阻塞到位
    if tag != 0:
        print(f"FAIL: rm_movej 返回 {tag}")
        arm.rm_delete_robot_arm()
        sys.exit(1)

    mid = read_joints(arm)
    print("中途  :", [f"{j:.2f}" for j in mid])
    err1 = max(abs(a - b) for a, b in zip(mid, target))
    print(f"home 段最大误差: {err1:.3f}°")
    if err1 >= 1.0:
        print("FAIL: home 段误差超 1°")
        arm.rm_delete_robot_arm()
        sys.exit(1)

    # 去 grasp1（有真实位移）再回 home，验证实际运动能力
    with open("robot_config.json") as f:
        g1 = json.load(f)["default_traj_js"]["grasp1"]
    grasp1 = [g1[f"J{i}"] for i in range(1, 7)]
    tag = arm.rm_movej(grasp1, SPEED, 0, 0, 1)
    if tag != 0:
        print(f"FAIL: 去 grasp1 返回 {tag}")
        arm.rm_delete_robot_arm()
        sys.exit(1)

    at_grasp = read_joints(arm)
    print("grasp1:", [f"{j:.2f}" for j in at_grasp])
    err2 = max(abs(a - b) for a, b in zip(at_grasp, grasp1))
    print(f"grasp1 段最大误差: {err2:.3f}°")

    tag = arm.rm_movej(target, SPEED, 0, 0, 1)
    if tag != 0:
        print(f"FAIL: 回 home 返回 {tag}")
        arm.rm_delete_robot_arm()
        sys.exit(1)
    back = read_joints(arm)
    print("回 home:", [f"{j:.2f}" for j in back])
    err3 = max(abs(a - b) for a, b in zip(back, target))

    ok = max(err1, err2, err3) < 1.0
    print(f"三段最大误差: {max(err1, err2, err3):.3f}°")
    print("PASS" if ok else "FAIL: 误差超 1°")

    arm.rm_delete_robot_arm()


if __name__ == "__main__":
    main()
