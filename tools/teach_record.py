#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
拖动示教录制工具
================
掰到位 -> 读真机关节角 -> 直接写进 robot_config.json, 不用手工抄。

robot_config.json 里位姿分两处存放, 按名字自动选:
    default_traj_js 内: home / grasp1-4 / place1-2
    顶层:               其余 (handover_pose / throw_to_trash_pose / desk_pose_* ...)

用法 (先停掉终端1 的 arm bringup, 它占着 192.168.1.18:8080):
    conda activate anygrasp
    python3 tools/teach_record.py --name grasp1   # 录一个
    python3 tools/teach_record.py                 # 连着录, 每次回车确认, q 退出
"""

import argparse
import json
import math
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(ROOT, "robot_config.json")
URDF_PATH = os.path.join(
    ROOT, "dependence/smart_pick_and_place_ws/src/rm_description/urdf/SingleArm/rm65.urdf")
ARM_IP, ARM_PORT = "192.168.1.18", 8080

# 与 Twin 的 z_min_threshold 一致: 横向转移的安全高度
MIN_SAFE_Z = 0.10

# 写进 default_traj_js 的名字, 其余写顶层
IN_TRAJ_JS = {"home", "grasp1", "grasp2", "grasp3", "grasp4", "place1", "place2"}

_Z_BODY = None


def _z_body():
    """懒加载一个 PyBullet DIRECT 客户端 + rm65 模型 (只读, 不碰真机)"""
    global _Z_BODY
    if _Z_BODY is None:
        import pybullet as p
        import pybullet_data
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        _Z_BODY = p.loadURDF(URDF_PATH, useFixedBase=True)
    return _Z_BODY


def min_z_of(joints):
    """
    把位姿摆出来, 返回 (最低点Z, 最低点link名), 单位米, 桌面 = 0。

    位姿间转移走的是裸关节插值, 没有任何 Z 检查 —— 端点贴在桌面上, 转移时夹爪
    就会蹭桌面。所以录完立刻报一下离桌面多远。没装 pybullet 时返回 (None, None)。
    """
    try:
        import pybullet as p
    except ImportError:
        return None, None
    uid = _z_body()
    for j, angle in enumerate(joints):
        p.resetJointState(uid, j, angle * math.pi / 180)
    low, who = 1e9, None
    for link in range(p.getNumJoints(uid)):
        z = p.getAABB(uid, link)[0][2]
        if z < low:
            low, who = z, p.getJointInfo(uid, link)[12].decode()
    return low, who


def write_pose(config, name, joints):
    """把关节角(度)写进 config 的对应位置, 返回写入的 pose dict"""
    pose = {"J%d" % (i + 1): round(float(j), 3) for i, j in enumerate(joints)}
    if name in IN_TRAJ_JS:
        config["default_traj_js"][name] = pose
    else:
        if name not in config:
            print("[warn] %s 不是已知位姿名, 将新建顶层字段" % name)
        config[name] = pose
    with open(CONFIG_PATH, "w") as f:
        json.dump(config, f, indent=4, ensure_ascii=False)
        f.write("\n")
    return pose


def check_limits(joints, lo, hi):
    """返回超限的关节描述列表"""
    return ["J%d=%.1f (限 %d..%d)" % (i + 1, j, lo[i], hi[i])
            for i, j in enumerate(joints) if not lo[i] <= j <= hi[i]]


def record_once(arm, config, name, lo, hi):
    # 该固件(V1.7.1)上 rm_get_current_arm_state 返回 145/150 且解析失败, 只能用 rm_get_joint_degree
    arm.rm_get_joint_degree()  # 丢掉 drag 开始后的第一帧
    tag, joints = arm.rm_get_joint_degree()
    if tag != 0:
        print("[error] 读关节角失败, tag=%d" % tag)
        return False
    if all(j == 0 for j in joints):
        print("[error] 关节角全 0, 多半是别的进程占着 SDK, 检查终端1 的 bringup 是否已停")
        return False

    print("  关节角(度): " + "  ".join("J%d=%.2f" % (i + 1, j) for i, j in enumerate(joints)))

    bad = check_limits(joints, lo, hi)
    if bad:
        print("[warn] 超出行程: " + ", ".join(bad))

    low, who = min_z_of(joints)
    if low is not None:
        note = ""
        if low < MIN_SAFE_Z:
            note = "  <== 低于 %.2fm, 位姿间横向转移会扫到桌面 (放桌面/低观察位姿可以忽略)" % MIN_SAFE_Z
        print("  最低点 %.3f m (%s)%s" % (low, who, note))

    pose = write_pose(config, name, joints)
    print("[ok] %s 已写入 robot_config.json: %s" % (name, json.dumps(pose, ensure_ascii=False)))
    return True


def main():
    ap = argparse.ArgumentParser(description="拖动示教录制, 直接写 robot_config.json")
    ap.add_argument("--name", "-n", help="位姿名; 不传则连续录制")
    ap.add_argument("--sensitivity", "-s", type=int, default=None,
                    help="拖动灵敏度 0-100 (越小越沉), 默认不动")
    args = ap.parse_args()

    from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

    arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
    handle = arm.rm_create_robot_arm(ARM_IP, ARM_PORT, level=3)
    if handle.id == -1:
        print("[error] 连不上机械臂 %s:%d" % (ARM_IP, ARM_PORT))
        return 1

    tag, lo = arm.rm_get_joint_min_pos()
    tag2, hi = arm.rm_get_joint_max_pos()
    if tag != 0 or tag2 != 0:
        print("[error] 读关节行程失败")
        return 1
    print("真机行程(度): " + "  ".join("J%d %.0f..%.0f" % (i + 1, lo[i], hi[i])
                                      for i in range(len(lo))))

    if args.sensitivity is not None:
        print("[drag] 设灵敏度 %d -> tag %d" % (args.sensitivity,
                                                arm.rm_set_drag_teach_sensitivity(args.sensitivity)))

    tag = arm.rm_start_drag_teach(0)
    print("[drag] 进入拖动示教 -> tag %d (0=成功, 现在可以用手掰臂了)" % tag)
    if tag != 0:
        # 示教器已开拖动模式时这里也会 tag 1, 臂照样能掰, 继续读
        print("[hint] tag!=0: 示教器可能已开着拖动模式 (那就直接掰); "
              "若臂是硬的, 查是否未使能 / 急停按下")

    with open(CONFIG_PATH) as f:
        config = json.load(f)

    try:
        names = [args.name] if args.name else None
        while True:
            if names:
                name = names[0]
            else:
                name = input("\n位姿名 (q 退出): ").strip()
                if name.lower() == "q" or not name:
                    break
            input("把臂掰到 %s 的目标姿态, 然后回车: " % name)
            record_once(arm, config, name, lo, hi)
            if names:
                break
    except KeyboardInterrupt:
        print("\n[drag] 中断")
    finally:
        if tag == 0:  # 只关自己开的, 示教器开的留给示教器
            print("[drag] 退出拖动示教 -> tag %d" % arm.rm_stop_drag_teach())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
