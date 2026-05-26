#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Grasp skill -- extracted from Planner.grasp phase in planner.py.

Cycle through observation positions, detect the target object with
AnyGrasp + YOLO-World filtering, validate reachability via the Twin
service, and execute the best grasp.

Usage (CLI):
    echo '{"object":"orange"}' | python3 run_skill.py grasp

    # or programmatically:
    from skills.base import get_skill
    GraspSkill = get_skill("grasp")
    skill = GraspSkill()
    skill.run(object="orange")
"""

import os
import copy
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint

from skills.base import Skill, register_skill
from core.transforms import (
    graspcam2pixel,
    self_rotation_np,
    rpy_to_vector,
)


@register_skill("grasp")
class GraspSkill(Skill):
    """
    Atomic grasp skill.

    Cycles through observation positions (grasp1..grasp4), captures RGB-D,
    runs AnyGrasp pose generation, filters with YOLO-World, transforms poses
    to world frame, and executes the best reachable grasp via the Twin service.
    """

    def __init__(self, **kw):
        super().__init__(**kw)
        self.rgb = None
        self.depth = None

    # ------------------------------------------------------------------
    # YOLO-World filtering
    # ------------------------------------------------------------------
    def filtering_pose(self, anygrasp_pose, class_name="", image=""):
        detections = self.perception.detect_objects(image, class_name, conf=0.2)

        grasp_points, grasp_pose_cam = graspcam2pixel(anygrasp_pose)
        valid_indices = set()
        final_grasps = []
        valid_boxes = []
        ans = False

        if len(detections):
            det = detections[0][:4]
            x1, y1, x2, y2 = det
            valid_boxes.append((x1, y1, x2, y2))
            for i, grasp_p in enumerate(grasp_points):
                if grasp_p[0] > x1 - 20 and grasp_p[0] < x2 + 20 and \
                   grasp_p[1] > y1 - 20 and grasp_p[1] < y2 + 20:
                    valid_indices.add(i)

            if len(valid_indices):
                sorted_indices = sorted(list(valid_indices))
                class_name_list = [cls for cls in class_name.split(",")]
                for i in sorted_indices:
                    g_pose = grasp_pose_cam[i]
                    final_grasps.append(g_pose)
                cprint(
                    f"*********** Class name {class_name_list} ****************** "
                    f"Grasp pose number: {len(final_grasps)} ******************",
                    "red",
                )
                ans = True
            else:
                cprint(f"Found objects ({class_name}) but NO grasp points inside them.", "yellow")
        else:
            cprint(f"No object detected for class: {class_name}", "yellow")

        return final_grasps if ans else []

    # ------------------------------------------------------------------
    # Pose transformation helpers
    # ------------------------------------------------------------------
    def transform_x_axis(self, transformed_pose_world):
        r, p, y = R.from_matrix(transformed_pose_world[:3, :3]).as_euler("xyz", degrees=False)
        x_axis_rotated = rpy_to_vector(r, p, y, axis=[1, 0, 0])
        y_axis_world = np.array([0, 1, 0])
        cos_theta = np.dot(x_axis_rotated, y_axis_world) / (
            np.linalg.norm(x_axis_rotated) * np.linalg.norm(y_axis_world)
        )
        if cos_theta > 0:
            transformed_pose_world = transformed_pose_world @ np.array([
                [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]
            ])
        return transformed_pose_world

    def transform_anygrasp_pose(self, anygrasp_pose, _visualization=True):
        try:
            eval_score = {}
            for id, data in enumerate(anygrasp_pose[:]):
                grasp_translation = data["trans"]
                grasp_rotation_matrix = data["rotation_matrix"]

                grasp_transformation_matrix = np.eye(4, 4)
                grasp_transformation_matrix[:3, :3] = grasp_rotation_matrix
                grasp_transformation_matrix[:3, 3] = grasp_translation

                self_pose_matrix = np.array([
                    [0, 1, 0, 0], [-1, 0, 0, 0],
                    [0, 0, 1, 0], [0, 0, 0, 1],
                ])
                self_pose_matrix = self_rotation_np(self_pose_matrix)

                transformed_pose_camera = grasp_transformation_matrix @ self_pose_matrix
                transformed_pose_world = self.T_base_to_cam @ transformed_pose_camera
                transformed_pose_world = self.transform_x_axis(transformed_pose_world)

                eval_score[id] = {}
                eval_score[id]["score"] = data["score"]
                eval_score[id]["transformed_pose_world"] = transformed_pose_world
                eval_score[id]["original_pose"] = anygrasp_pose[id]

            final_grasp_pose_data = sorted(
                eval_score.items(), key=lambda x: x[1]["score"], reverse=True
            )
            return [
                final_grasp_pose_data[i][1]["transformed_pose_world"]
                for i in range(len(final_grasp_pose_data))
            ]
        except Exception:
            return []

    # ------------------------------------------------------------------
    # Grasp execution
    # ------------------------------------------------------------------
    def execute_grasping_twin_js_2(self, grasping_pose_world_hand, idx=None):
        self.control_hand(cmd_type="open")
        basic_mat = np.eye(4)
        basic_mat[2, 3] -= 0.02
        preparasion_grasping_pos = grasping_pose_world_hand @ basic_mat
        preparasion_grasping_pos[2, 3] += 0.02
        preparasion_grasping_pos = preparasion_grasping_pos @ self.T_hand_effector_to_arm_endlink
        preparasion_grasping_pos_position = preparasion_grasping_pos[:3, 3]
        preparasion_grasping_pos_orientation = R.from_matrix(
            preparasion_grasping_pos[:3, :3]
        ).as_quat()

        basic_mat = np.eye(4)
        execution_grasping_pos = grasping_pose_world_hand @ basic_mat
        print("=-============================", execution_grasping_pos[2, 3])
        execution_grasping_pos[2, 3] = max(execution_grasping_pos[2, 3], 0.042)
        execution_grasping_pos = execution_grasping_pos @ self.T_hand_effector_to_arm_endlink
        execution_grasping_pos_position = execution_grasping_pos[:3, 3]
        execution_grasping_pos_orientation = R.from_matrix(
            execution_grasping_pos[:3, :3]
        ).as_quat()

        default_traj_js_rad = [
            data / 180 * np.pi
            for data in self.config.default_traj_js[idx].values()
        ]
        cnfg = {
            "target_pose": [
                [preparasion_grasping_pos_position[0], preparasion_grasping_pos_position[1],
                 preparasion_grasping_pos_position[2], preparasion_grasping_pos_orientation[0],
                 preparasion_grasping_pos_orientation[1], preparasion_grasping_pos_orientation[2],
                 preparasion_grasping_pos_orientation[3]],
                [execution_grasping_pos_position[0], execution_grasping_pos_position[1],
                 execution_grasping_pos_position[2], execution_grasping_pos_orientation[0],
                 execution_grasping_pos_orientation[1], execution_grasping_pos_orientation[2],
                 execution_grasping_pos_orientation[3]],
            ],
            "current_js": default_traj_js_rad,
            "struct": "left_arm",
        }
        rsp = self.send_cmd_twin(self.twin, {
            "srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg
        })
        state = rsp["value"]

        if state:
            trajectory_grasping = rsp["info"]["trajectory"]
            trajectory_grasping = np.array(copy.deepcopy(trajectory_grasping)) / np.pi * 180
            self.control_arm(trajectory=trajectory_grasping, speed=20)
            cprint("=============== Reach grasping pose =============")
            self.control_hand(cmd_type="close")
            cprint("=============== Close hand =============")
            time.sleep(0.5)
            self.control_arm(pose_type=idx, speed=30)
            cprint("=============== Reach post grasping pose =============")
            return True
        else:
            cprint("********************* Grasp pose not reachable *********************", "red")
            return False

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def run(self, **kwargs):
        """
        Execute the grasp skill.

        Keyword Args:
            object (str): Name of the object to grasp (YOLO-World class name).
                          Required.

        Returns:
            bool: True if grasp succeeded, False otherwise.
        """
        obj = kwargs.get("object")
        if obj is None:
            cprint("[grasp] Missing required kwarg: object", "red")
            return False

        self.control_hand(cmd_type="close")
        check = False

        for key in self.config.default_traj_js:
            if "grasp" not in key:
                continue

            self.control_arm(pose_type=key, speed=30)
            self.rgb, self.depth = self.get_camera_obs()
            cprint(
                f"G=================== Save current rgb and depth observations: "
                f"{self.save_path} ===================",
                "cyan",
            )
            anygrasp_pose = self.perception.detect_grasps(self.rgb, self.depth)
            cprint(
                f"G=================== Generate the grasping pose and save it "
                f"in file: {self.save_path}/result.json ===================",
                "cyan",
            )
            self.save_current_transformation()
            if not anygrasp_pose:
                continue

            filtering_grasping_pose = self.filtering_pose(
                anygrasp_pose, class_name=obj, image=self.rgb
            )
            if not filtering_grasping_pose:
                continue

            grasping_pose_world = self.transform_anygrasp_pose(
                filtering_grasping_pose, _visualization=False
            )
            if not len(grasping_pose_world):
                continue

            for i in range(len(grasping_pose_world)):
                cprint(
                    f"=================== Checking pose: {i+1} / "
                    f"{len(grasping_pose_world)} ===================",
                    "yellow",
                )
                check = self.execute_grasping_twin_js_2(
                    grasping_pose_world[i], idx=key
                )
                if check:
                    break

            if check:
                break

        if not check:
            cprint("G=================== Grasping task failed: 所有观测位均未能生成可达的抓取轨迹 ===================", "red")
            cprint("G=================== 可能原因: 目标不可达 / 碰撞 / 物品位置不佳 ===================", "yellow")
            return False

        cprint("G=================== Successfully completed the grasping task ===================", "green")
        return True
