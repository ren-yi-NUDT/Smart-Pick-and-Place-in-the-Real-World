#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Grasp-to-drawer skill -- dual-arm coordination example.

The left arm (dexterous hand) grasps an object from the table, the right
arm (gripper) opens a drawer, both arms perform a handover, and the right
arm places the object into the drawer.

Usage (CLI):
    echo '{"object":"orange","container":"drawer1"}' | python3 run_skill.py grasp_to_drawer

    # or programmatically:
    from skills.base import get_skill
    Skill = get_skill("grasp_to_drawer")
    skill = Skill()
    skill.run(object="orange", container="drawer1")
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint

from core.twin_client import TwinClient
from skills.base import DualArmSkill, register_skill


@register_skill("grasp_to_drawer")
class GraspToDrawer(DualArmSkill):
    """Left arm grasps object, handover to right arm, right arm places into drawer."""

    def run(self, **kwargs):
        data = kwargs if kwargs.get("object") else self.json_parser.get_command()
        obj = data.get("object", "orange")

        cprint(f"[grasp_to_drawer] 目标物体: {obj}", "cyan")

        # ── Phase 1: Left arm grasps the object ──
        cprint("[grasp_to_drawer] 阶段1: 手臂抓取物体", "yellow")
        if not self._grasp_with_left_arm(obj):
            cprint("[grasp_to_drawer] 抓取失败", "red")
            return False

        # ── Phase 2: Right arm opens the drawer (pre-recorded motion) ──
        cprint("[grasp_to_drawer] 阶段2: 夹爪臂打开抽屉", "yellow")
        self.right.arm.move_to_named_pose(
            self.right.get_pose("open_drawer"), speed=15
        )

        # ── Phase 3: Dual-arm handover ──
        cprint("[grasp_to_drawer] 阶段3: 双臂交接", "yellow")
        self.handover(from_side="left", to_side="right")

        # ── Phase 4: Right arm places into drawer (pre-recorded motion) ──
        cprint("[grasp_to_drawer] 阶段4: 放入抽屉", "yellow")
        self.right.arm.move_to_named_pose(
            self.right.get_pose("place_into_drawer"), speed=15
        )
        self.right.hand.open()

        cprint("[grasp_to_drawer] 完成", "green")
        return True

    # ------------------------------------------------------------------
    # Internal: visual grasp with left arm
    # ------------------------------------------------------------------
    def _grasp_with_left_arm(self, obj_name):
        """Cycle through observation poses, detect the object, and grasp it.

        This is a simplified / placeholder version of the full grasp pipeline
        (see ``skills/grasp.py`` for the production implementation).  It
        demonstrates the DualArmSkill / ArmSide API usage while keeping the
        trajectory generation logic illustrative.
        """
        for grasp_pose_name in ["grasp1", "grasp2", "grasp3", "grasp4"]:
            try:
                pose = self.left.get_pose(grasp_pose_name)
            except KeyError:
                continue

            self.left.arm.move_to_named_pose(pose, speed=20)

            # Capture and cache transforms
            rgb, depth = self.camera.get_rgbd()
            self.save_current_transformation()

            # Run AnyGrasp + YOLO-World filtering
            anygrasp_pose = self._run_anygrasp(rgb, depth)
            if not anygrasp_pose:
                continue

            filtered = self._filter_grasps(anygrasp_pose, obj_name, rgb)
            if not filtered:
                continue

            # Transform to world frame and attempt trajectory
            world_poses = self._transform_to_world(filtered)
            if not world_poses:
                continue

            for gp in world_poses:
                traj = self._plan_grasp_trajectory(gp)
                if traj is None:
                    continue

                # Convert rad -> deg and execute
                trajectory_deg = np.degrees(traj).tolist()
                self.left.arm.execute_trajectory(trajectory_deg, speed=15)
                self.left.hand.close()

                # Return to observation pose
                self.left.arm.move_to_named_pose(pose, speed=20)
                return True

        return False

    # ------------------------------------------------------------------
    # Placeholder helpers (illustrative -- not production-ready)
    # ------------------------------------------------------------------
    def _run_anygrasp(self, rgb, depth):
        """Run AnyGrasp on the RGB-D pair via the long-running server.

        Returns raw grasp candidates or [].
        """
        try:
            return self.perception.detect_grasps(rgb, depth) or []
        except Exception as e:
            cprint(f"[grasp_to_drawer] AnyGrasp failed: {e}", "red")
            return []

    def _filter_grasps(self, anygrasp_pose, class_name, image):
        """Filter AnyGrasp candidates by YOLO-World bounding box.

        Returns a list of grasp pose dicts that fall inside the detection box.
        """
        from core.transforms import graspcam2pixel

        class_name_list = [cls.strip() for cls in class_name.split(",")]
        detections = self.perception.detect_objects(image, class_name_list, conf=0.2)
        if not detections:
            cprint(f"[grasp_to_drawer] 未检测到: {class_name_list}", "yellow")
            return []

        det = detections[0][:4]
        x1, y1, x2, y2 = det
        grasp_points, grasp_pose_cam = graspcam2pixel(anygrasp_pose)

        valid = []
        for i, pt in enumerate(grasp_points):
            if (x1 - 20) < pt[0] < (x2 + 20) and (y1 - 20) < pt[1] < (y2 + 20):
                valid.append(grasp_pose_cam[i])

        if valid:
            cprint(
                f"[grasp_to_drawer] 检测到 {class_name_list}, "
                f"有效抓取: {len(valid)}",
                "green",
            )
        else:
            cprint(
                f"[grasp_to_drawer] 检测到 {class_name_list} 但无有效抓取点",
                "yellow",
            )
        return valid

    def _transform_to_world(self, grasp_poses):
        """Transform filtered camera-frame grasp poses to world frame.

        Returns a list of 4x4 numpy arrays (homogeneous transforms).
        """
        from core.transforms import self_rotation_np

        results = []
        for gp in grasp_poses:
            translation = gp["trans"]
            rotation = gp["rotation_matrix"]
            T_cam = np.eye(4)
            T_cam[:3, :3] = rotation
            T_cam[:3, 3] = translation

            self_pose = np.array([
                [0, 1, 0, 0], [-1, 0, 0, 0],
                [0, 0, 1, 0], [0, 0, 0, 1],
            ])
            self_rot = self_rotation_np(self_pose)
            T_world = self.T_base_to_cam @ (T_cam @ self_rot)

            # Ensure x-axis convention (same as GraspSkill.transform_x_axis)
            r, p, y = R.from_matrix(T_world[:3, :3]).as_euler("xyz", degrees=False)
            from core.transforms import rpy_to_vector
            x_axis = rpy_to_vector(r, p, y, axis=[1, 0, 0])
            if np.dot(x_axis, [0, 1, 0]) > 0:
                T_world = T_world @ np.diag([-1, -1, 1, 1]).astype(float)

            results.append(T_world)
        return results

    def _plan_grasp_trajectory(self, grasp_pose_world):
        """Build a pre-grasp + grasp trajectory via the twin service.

        Returns the trajectory array (radians) or None on failure.
        """
        # Pre-grasp: offset 8 cm back along approach direction, 2 cm up
        T_pre = grasp_pose_world.copy()
        T_pre[2, 3] -= 0.08
        T_pre[2, 3] += 0.02

        T_pre_link = T_pre @ self.T_hand_effector_to_arm_endlink
        T_grasp_link = grasp_pose_world @ self.T_hand_effector_to_arm_endlink
        T_grasp_link[2, 3] = max(T_grasp_link[2, 3], 0.042)

        prep_pos = T_pre_link[:3, 3]
        prep_orn = R.from_matrix(T_pre_link[:3, :3]).as_quat()
        grasp_pos = T_grasp_link[:3, 3]
        grasp_orn = R.from_matrix(T_grasp_link[:3, :3]).as_quat()

        # Default joint state as current_js (radians)
        default_js_rad = [
            v / 180 * np.pi
            for v in self.left.get_pose("grasp1").values()
        ]

        cnfg = TwinClient.build_config_grasp(
            prep_pos, prep_orn,
            grasp_pos, grasp_orn,
            current_js_rad=default_js_rad,
            struct="left_arm",
        )
        resp = self.twin.generate_trajectory2(cnfg)

        if not resp.get("value", False):
            info = resp.get("info", {})
            cprint(
                f"[grasp_to_drawer] 轨迹不可达: "
                f"collided={info.get('is_collided', '?')}",
                "red",
            )
            return None

        trajectory = np.array(resp["info"]["trajectory"])
        return trajectory
