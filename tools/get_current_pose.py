#!/usr/bin/env python3
"""
获取当前机械臂的真实关节参数
运行此脚本前，确保机械臂已上电并可连接
"""

from Robotic_Arm.rm_robot_interface import *
import json

def get_current_arm_state(arm_ip="192.168.1.19", arm_port=8080):
    """
    获取当前机械臂的关节状态和末端位姿

    Returns:
        dict: 包含关节角度(度)和末端位姿
    """

    # 初始化机械臂连接
    arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
    handle = arm.rm_create_robot_arm(arm_ip, arm_port, level=3)

    if handle.id == -1:
        print(f"错误: 无法连接机械臂 {arm_ip}:{arm_port}")
        return None

    print(f"成功连接机械臂, ID: {handle.id}")

    # 获取当前状态
    tag, arm_state = arm.rm_get_current_arm_state()

    if tag != 0:
        print("错误: 无法获取机械臂状态")
        arm.rm_delete_robot_arm()
        return None

    result = {
        "joint_angles_deg": arm_state["joint"],  # 7个关节角度，单位：度
        "end_pose": arm_state["pose"],            # 末端位姿 [x, y, z, rx, ry, rz]
    }

    print("\n" + "="*60)
    print("当前机械臂状态:")
    print("="*60)

    # 打印关节角度
    joints = arm_state["joint"]
    print("\n关节角度 (度):")
    for i, j in enumerate(joints):
        print(f"  J{i+1}: {j:.3f}°")

    # 打印末端位姿
    pose = arm_state["pose"]
    print("\n末端位姿:")
    print(f"  位置: x={pose[0]:.4f}, y={pose[1]:.4f}, z={pose[2]:.4f} (米)")
    print(f"  姿态: rx={pose[3]:.4f}, ry={pose[4]:.4f}, rz={pose[5]:.4f} (弧度)")

    # 生成配置格式
    print("\n" + "="*60)
    print("复制以下内容到 robot_config.json:")
    print("="*60)
    config_str = '''
    "pose": {
        "J1": %.3f,
        "J2": %.3f,
        "J3": %.3f,
        "J4": %.3f,
        "J5": %.3f,
        "J6": %.3f,
        "J7": %.3f
    }
''' % tuple(joints)
    print(config_str)

    # 保持连接，不主动断开
    # arm.rm_delete_robot_arm()

    return result


if __name__ == "__main__":
    # 机械臂IP（根据实际情况修改）
    ARM_IP = "192.168.1.19"

    print("请将机械臂手动拖动到你想要的递送位置，然后按回车继续...")
    input()

    result = get_current_arm_state(arm_ip=ARM_IP)

    if result:
        print("\n获取位姿成功")
