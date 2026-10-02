#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""孪生坐标系自洽性回归检查。

跑法 (需要 PyBullet + URDF, 不需要 ROS / 不需要连真机):
    cd dependence/twin_inference && python3 test_frame_consistency.py

守的是三个已经踩过的坑, 每个都会让**所有**抓取候选被判不可达, 而且症状很隐蔽
(delta_rpy 只有零点几弧度、位置差 0.3m, 看起来像"够不着"而不是"坐标系错了"):

1. self 系锚点必须是 URDF root link (world 原点), 不能是 link_1 的质心系
   —— 后者跟着 J1 转、还高 0.133m。
2. 锚点也不能用 getBasePositionAndOrientation 直接拿 —— 那是基座的**质心**系,
   base_link 惯量原点偏了 0.060019m。
3. 末端 FK 必须取 getLinkState(...)[4:6] (URDF link 系), 不能取 [:2] (质心系),
   否则和 calculateInverseKinematics 的目标系差一个质心偏移。
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import pybullet as p

from robot import ErdaijiRobot, robot_base_pose, robot_to_world, world_to_robot
from twin import TwinTest2

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_DIR = os.path.join(
    HERE, "..", "smart_pick_and_place_ws", "src", "rm_description", "urdf", "SingleArm"
)
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))


class _Twin(TwinTest2):
    def __init__(self):
        self.vis = False
        self.camera_refresh_freq = 50
        self.robot = ErdaijiRobot(
            (0.0, 0.0, 0.0), (0, 0, 0),
            robot_path=os.path.join(URDF_DIR, "rm65.urdf"),
            config_path=os.path.join(URDF_DIR, "robot_config.json"),
            fixed_robot=True, vis=False, blocking_mode=False,
        )
        self.setup()


def main():
    twin = _Twin()
    a = twin.robot.robot_structs["rm65"]
    a.count_execute_cmd = 0

    # --- 坑 1+2: 锚点必须是 URDF root link, 也就是 world 原点 -------------
    base_pos, base_orn = robot_base_pose(a.robot_id)
    assert np.allclose(base_pos, 0.0, atol=1e-6), (
        f"self 系锚点应为 world 原点, 实得 {np.round(base_pos, 6)} —— "
        "又用回质心系了? (getLinkState(id,0)[:2] 或 getBasePositionAndOrientation)"
    )
    assert np.allclose(base_orn, [0, 0, 0, 1], atol=1e-6), f"锚点姿态应为单位四元数, 实得 {base_orn}"

    # --- 坑 3: 末端 FK 取 [4:6], 且 self/world 往返恒等 --------------------
    cfg = json.load(open(os.path.join(PROJECT_ROOT, "robot_config.json")))
    for name in ("grasp1", "grasp2", "grasp3", "grasp4", "place1", "place2"):
        pose = cfg["default_traj_js"].get(name)
        if pose is None:
            continue
        q = [math.radians(pose[f"J{i}"]) for i in range(1, 7)]
        a.cmd_queue = []
        a.reset_by_joint_states(q)
        a.move_joint(q)
        twin.step()

        pos_world, orn_world = a.get_end_link_pose_world()
        pos_self, orn_self = a.get_end_link_pose_self()
        err_self = np.linalg.norm(np.array(pos_self) - np.array(pos_world))
        assert err_self < 1e-4, (
            f"{name}: self 系与 world 系应重合 (锚点在原点), 实测差 {err_self*1000:.2f}mm"
        )

        # 往返: self -> world -> self
        w = robot_to_world(a.robot_id, pos_self, orn_self)
        s = world_to_robot(a.robot_id, *w)
        assert np.linalg.norm(np.array(s[0]) - np.array(pos_self)) < 1e-9, f"{name}: 往返位置不闭合"
        assert abs(float(np.dot(s[1], orn_self))) > 1 - 1e-9, f"{name}: 往返姿态不闭合"

        # 自己到自己的 IK 必须命中 —— 这是 check_reach 的阈值量级
        # check_reach 必须在 step 之前: POSITION_CONTROL 撑不住重力, 一步 stepSimulation
        # 就能把末端甩开 17mm, 那是"重力把臂拖到哪了", 不是"IK 解能不能到位"。
        a.reset_by_ee_pose_self(pos_self, orn_self, js=q)
        is_reach, dxyz, drpy = a.check_reach(pos_self, orn_self, xyz_threshold=0.01, rpy_threshold=0.01)
        twin.step()
        assert is_reach, f"{name}: 录制位姿自己到自己判不可达 ({dxyz*1000:.2f}mm / {drpy:.4f}rad)"
        print(f"  {name}: 自洽 {dxyz*1000:.3f}mm / {drpy:.5f}rad  OK")

    # --- 坑 4: 发给真机的关节角必须在真机限位内 ---------------------------
    # PyBullet 的 resetJointState 不看限位, 所以 IK 给出 J4=-210° 时仿真 FK 照样准
    # (revolute 关节是周期的), check_reach 报 0.1mm"可达" —— 真机 J4 却卡在 ±178°,
    # 夹爪落在离目标几十厘米的地方。实测就是这么"差很远"地抓空的。
    g3 = [math.radians(cfg["default_traj_js"]["grasp3"][f"J{i}"]) for i in range(1, 7)]

    assert a.wrap_joint_limits([0, 0, 0, math.radians(-210.5), 0, 0]) is not None, \
        "J4=-210.5° 应能 2π 回卷到限位内"
    assert a.wrap_joint_limits([0, 0, 0, math.radians(-178.9), 0, 0]) is None, \
        "J4=-178.9° 落在限位死区, 应折不回去"

    # ZYZ 翻转 (J4±π, -J5, J6±π) 必须是同一个末端位姿
    a.reset_by_joint_states(g3)
    p0, o0 = a.get_end_link_pose_world()
    flip_raw = list(g3)
    flip_raw[3] += math.pi
    flip_raw[4] = -flip_raw[4]
    flip_raw[5] += math.pi
    flip = a.enforce_joint_limits(flip_raw)
    assert flip is not None and a.in_joint_limits(flip), f"ZYZ 翻转解应在限位内, 实得 {flip}"
    a.reset_by_joint_states(flip)
    p1, o1 = a.get_end_link_pose_world()
    assert np.linalg.norm(np.array(p1) - np.array(p0)) < 1e-5, "ZYZ 翻转后位置应不变"
    assert abs(float(np.dot(o1, o0))) > 1 - 1e-6, "ZYZ 翻转后姿态应不变"

    # 端到端: 孪生发出的轨迹里不能有越限路点, 也不能有换腕分支的跳变。
    # 目标用 grasp3 自己的末端姿态 (只改位置), 这是"从观察位姿垂直下探"的典型形状,
    # 位形良好; 拿一个凭空捏的姿态去测会先卡在 IK 病态上, 测不到限位这条路径。
    a.reset_by_joint_states(g3)
    _, orn = a.get_end_link_pose_self()
    # 目标点是 link_6 (末端链系), 夹爪还往下伸 0.113m, 所以 z 不能给太低,
    # 否则先被 Z 高度检查按"插进桌面"挡掉。
    checked = rejected = 0
    for x in np.arange(-0.45, -0.10, 0.05):
        for y in np.arange(-0.20, 0.30, 0.05):
            cnfg = {"struct": "rm65", "current_js": list(g3),
                    "target_pose": [[x, y, 0.20] + list(orn)]}
            ok, traj, _, _ = twin.linear_traj_generation2(cnfg)
            if not ok:
                rejected += 1
                continue
            checked += 1
            arr = np.array(traj)
            assert a.in_joint_limits(arr[0]), f"({x:.2f},{y:.2f}): 轨迹首点越限"
            lo, hi = np.array(a.ll_robot[:6]), np.array(a.ul_robot[:6])
            assert np.all(arr >= lo - 1e-6) and np.all(arr <= hi + 1e-6), \
                f"({x:.2f},{y:.2f}): 轨迹里有越限路点\n{np.degrees(arr).round(1)}"
            jump = np.abs(np.diff(arr, axis=0)).max()
            assert jump <= math.radians(90), \
                f"({x:.2f},{y:.2f}): 相邻路点跳变 {math.degrees(jump):.1f}° (换腕分支了)"
    print(f"  关节限位: 网格 {checked + rejected} 个目标, {checked} 条轨迹全部在限位内且连续, "
          f"{rejected} 个按不可达拒发")
    assert checked > 0, "一个可行目标都没有, 检查是不是把全部目标都误判成不可达了"

    print("坐标系自洽性检查全部通过")


if __name__ == "__main__":
    main()
