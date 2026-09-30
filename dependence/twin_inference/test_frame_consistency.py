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
        a.reset_by_ee_pose_self(pos_self, orn_self, js=q)
        twin.step()
        is_reach, dxyz, drpy = a.check_reach(pos_self, orn_self, xyz_threshold=0.01, rpy_threshold=0.01)
        assert is_reach, f"{name}: 录制位姿自己到自己判不可达 ({dxyz*1000:.2f}mm / {drpy:.4f}rad)"
        print(f"  {name}: 自洽 {dxyz*1000:.3f}mm / {drpy:.5f}rad  OK")

    print("坐标系自洽性检查全部通过")


if __name__ == "__main__":
    main()
