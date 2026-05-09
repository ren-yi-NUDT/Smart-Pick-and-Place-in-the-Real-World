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
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from datetime import datetime
from PIL import Image
from scipy.spatial.transform import Rotation as R
from termcolor import cprint

from skills.base import Skill, register_skill
from core.transforms import (
    graspcam2pixel,
    self_rotation_np,
    rpy_to_vector,
    transform_world_to_camera,
    self_rotation_inv,
    visualization,
    pixel_to_camera_point,
    pixel_to_camera_point2,
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
        self.point_cloud = None
        self.self_pose_matrix = None
        self.final_grasp_pose_data = []
        self.default_traj_js_rad = [
            v / 180 * np.pi
            for v in self.config.default_traj_js["grasp1"].values()
        ]

    # ------------------------------------------------------------------
    # AnyGrasp pose generation
    # ------------------------------------------------------------------
    def get_pose_and_save(self, rgb, depth):
        """Run AnyGrasp inference on an RGB-D pair."""
        try:
            from anygrasp_sdk.grasp_detection.anygrasp_get_poses import (
                anygrasp_get_poses,
            )
            from core.config import DEFAULT_ANYGRASP_CHECKPOINT
            anygrasp_pose, self.point_cloud = anygrasp_get_poses(
                DEFAULT_ANYGRASP_CHECKPOINT, rgb, depth
            )
            return anygrasp_pose
        except Exception:
            return False

    # ------------------------------------------------------------------
    # YOLO-World filtering
    # ------------------------------------------------------------------
    def filtering_pose(self, anygrasp_pose, class_name="", image="", return_label=False, vis=True):
        class_name_list = [cls for cls in class_name.split(",")]
        detections = self.perception.detect_objects(image, class_name_list, conf=0.2)

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
                for i in sorted_indices:
                    g_pose = grasp_pose_cam[i]
                    if return_label:
                        if isinstance(g_pose, dict):
                            g_pose["label"] = class_name_list
                    final_grasps.append(g_pose)
                cprint(
                    f"*********** Class name {class_name_list} ****************** "
                    f"Grasp pose number: {len(final_grasps)} ******************",
                    "red",
                )
                ans = True
            else:
                cprint(
                    f"Found objects ({class_name_list}) but NO grasp points inside them.",
                    "yellow",
                )
        else:
            cprint(f"No object detected for class: {class_name_list}", "yellow")

        if vis:
            plt.figure(figsize=(10, 8))
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plt.imshow(image)
            ax = plt.gca()

            for box in valid_boxes:
                bx1, by1, bx2, by2 = box
                ax.add_patch(
                    plt.Rectangle((bx1, by1), bx2 - bx1, by2 - by1,
                                  fill=False, color="red", linewidth=2)
                )
                ax.text(bx1, by1 - 5, str(class_name_list), color="red",
                        fontsize=10, weight="bold")

            if final_grasps:
                for idx in valid_indices:
                    plt.plot(grasp_points[idx][0], grasp_points[idx][1],
                             "g*", markersize=8, label="Valid")
            else:
                if len(grasp_points) > 0:
                    plt.plot(grasp_points[:, 0], grasp_points[:, 1],
                             "b.", markersize=6, alpha=0.5)
            plt.title(f"{class_name_list}: {len(valid_boxes)} objects, {len(final_grasps)} grasps")
            plt.axis("off")
            save_filename = f"filtered_rgb_{timestamp}_{class_name}.png"
            plt.savefig(os.path.join(self.save_path, save_filename))
            plt.close()

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
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
        return transformed_pose_world

    def publish_single_pose(self, transform_matrix, child_frame_id,
                            parent_frame_id="base_link"):
        import rospy
        from geometry_msgs.msg import TransformStamped
        if transform_matrix.shape != (4, 4):
            rospy.logerr("Input matrix must be 4x4.")
            return
        translation = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]
        try:
            quat = R.from_matrix(rotation_matrix).as_quat()
        except Exception as e:
            rospy.logerr(f"Cannot convert rotation matrix to quaternion: {e}")
            return
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform([t])
        rospy.loginfo(f"Published pose to RViz: {child_frame_id}, parent: {parent_frame_id}")

    def publish_grasp_transforms(self, parent_frame="base_link"):
        from geometry_msgs.msg import TransformStamped
        transforms_list = []
        for i, (original_id, item) in enumerate(self.final_grasp_pose_data[:]):
            pose_matrix = item["transformed_pose_world"]
            translation = pose_matrix[:3, 3]
            rotation_matrix = pose_matrix[:3, :3]
            quat = R.from_matrix(rotation_matrix).as_quat()
            t = TransformStamped()
            t.header.stamp = rospy.Time(0)
            t.header.frame_id = parent_frame
            t.child_frame_id = f"grasp_pose_hand_endeffector_{i}"
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            transforms_list.append(t)
        if transforms_list:
            self.tf_broadcaster.sendTransform(transforms_list)

    def transform_anygrasp_pose(self, anygrasp_pose, _visualization=True,
                                return_labels=False):
        try:
            import rospy
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
                self.self_pose_matrix = self_rotation_np(self_pose_matrix)

                transformed_pose_camera = grasp_transformation_matrix @ self.self_pose_matrix
                transformed_pose_world = self.T_base_to_cam @ transformed_pose_camera
                transformed_pose_world = self.transform_x_axis(transformed_pose_world)

                eval_score[id] = {}
                eval_score[id]["score"] = data["score"]
                eval_score[id]["transformed_pose_world"] = transformed_pose_world
                eval_score[id]["original_pose"] = anygrasp_pose[id]
                if return_labels:
                    eval_score[id]["label"] = data["label"]

            self.final_grasp_pose_data = sorted(
                eval_score.items(), key=lambda x: x[1]["score"], reverse=True
            )

            self.publish_grasp_transforms(parent_frame="base_link")

            if return_labels:
                return (
                    [self.final_grasp_pose_data[i][1]["transformed_pose_world"]
                     for i in range(len(self.final_grasp_pose_data))],
                    [self.final_grasp_pose_data[i][1]["label"]
                     for i in range(len(self.final_grasp_pose_data))],
                )
            else:
                return [
                    self.final_grasp_pose_data[i][1]["transformed_pose_world"]
                    for i in range(len(self.final_grasp_pose_data))
                ]
        except Exception:
            return [], None

    def visualization_3d_grasping_pose(self, grasping_pose_world,
                                       translation_matrix2=None):
        pose_matrix = grasping_pose_world
        print(
            f"Pose of grasping in root frame: {pose_matrix}\n"
            f"{R.from_matrix(pose_matrix[:3, :3]).as_euler('xyz', degrees=False)}"
        )
        pose_c = transform_world_to_camera(pose_matrix, self.T_base_to_cam)
        self_rot_inv = self_rotation_inv(self.self_pose_matrix)
        rot_g = np.dot(pose_c[:3, :3], self_rot_inv)
        translation_matrix = np.eye(4, 4)
        translation_matrix[0:3, 0:3] = rot_g
        translation_matrix[0:3, 3] = pose_c[:3, 3]
        if translation_matrix2 is None:
            visualization(self.point_cloud, translation_matrix)
        else:
            visualization(
                self.point_cloud,
                np.concatenate((translation_matrix, translation_matrix2), axis=0),
            )

    # ------------------------------------------------------------------
    # Twin-service helpers
    # ------------------------------------------------------------------
    def create_send_config_2(self, prep_pos, prep_orn, exec_pos, exec_orn,
                             current_js_pose=None, struct="left_arm"):
        if current_js_pose is None:
            self._config = {
                "target_pose": [
                    [prep_pos[0], prep_pos[1], prep_pos[2],
                     prep_orn[0], prep_orn[1], prep_orn[2], prep_orn[3]],
                    [exec_pos[0], exec_pos[1], exec_pos[2],
                     exec_orn[0], exec_orn[1], exec_orn[2], exec_orn[3]],
                ],
                "current_js": list(self.default_traj_js_rad),
                "struct": struct,
            }
        else:
            self._config = {
                "target_pose": [
                    [prep_pos[0], prep_pos[1], prep_pos[2],
                     prep_orn[0], prep_orn[1], prep_orn[2], prep_orn[3]],
                    [exec_pos[0], exec_pos[1], exec_pos[2],
                     exec_orn[0], exec_orn[1], exec_orn[2], exec_orn[3]],
                ],
                "current_js": list(current_js_pose),
                "struct": struct,
            }

    def create_twin_service(self, type=None, cnfg=None):
        cmd = {"srv": "twin_inference", "type": type, "cnfg": cnfg}
        resp = self.send_cmd_twin(self.twin, cmd)
        return resp

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

        self.create_send_config_2(
            preparasion_grasping_pos_position,
            preparasion_grasping_pos_orientation,
            execution_grasping_pos_position,
            execution_grasping_pos_orientation,
        )
        rsp = self.create_twin_service(type="trajectory_generation2", cnfg=self._config)
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
            twin_info = rsp.get("info", {})
            cprint(f"********************* Grasp pose not reachable: "
                   f"collided={twin_info.get('is_collided', '?')}, "
                   f"delta_xyz={twin_info.get('delta_xyz', '?')} *********************", "red")
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
            anygrasp_pose = self.get_pose_and_save(self.rgb, self.depth)
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
