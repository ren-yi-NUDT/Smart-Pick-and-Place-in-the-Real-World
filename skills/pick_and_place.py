import copy
import random
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core.transforms import graspcam2pixel, self_rotation_np, rpy_to_vector, pixel_to_camera_point2


@register_skill("pick_and_place")
class PickAndPlaceSkill(Skill):
    """Main pick-and-place pipeline: detect → grasp → place."""

    def run(self, **kwargs):
        if kwargs.get("object") and kwargs.get("container"):
            json_data = kwargs
        else:
            json_data = self.json_parser.get_command()
        if json_data is None:
            cprint("未收到有效的JSON输入", "red")
            return False

        cprint(f"=================== 1. Get JSON input: {json_data} ===================", "cyan")

        obj, container = json_data.get("object"), json_data.get("container")
        if obj is None or container is None:
            cprint("JSON输入缺少必需字段 (object 或 container)", "red")
            return False

        cprint(f"=================== 2. Parse input: Grasp {obj} and place it in the {container} ===================", "cyan")
        self.control_arm(pose_type="grasp1", speed=30)
        self.control_hand(cmd_type="close")

        # ---- Grasp phase ----
        check = False
        for key, value in self.config.default_traj_js.items():
            if "grasp" not in key:
                continue
            self.control_arm(pose_type=key, speed=30)
            self.rgb, self.depth = self.get_camera_obs()
            cprint(f"G=================== 3. Save current rgb and depth observations: {self.save_path} ===================", "cyan")

            anygrasp_pose = self.perception.detect_grasps(self.rgb, self.depth)
            cprint(f"G=================== 4. Generate the grasping pose and save it in file: {self.save_path}/result.json ===================", "cyan")
            self.save_current_transformation()
            if not anygrasp_pose:
                continue

            filtering_grasping_pose = self._filtering_pose(anygrasp_pose, class_name=obj, image=self.rgb)
            if not filtering_grasping_pose:
                continue

            grasping_pose_world = self._transform_anygrasp_pose(filtering_grasping_pose, _visualization=False)
            if not len(grasping_pose_world):
                continue

            for i in range(len(grasping_pose_world)):
                cprint(f"=================== Checking pose: {i+1} / {len(grasping_pose_world)} ===================", "yellow")
                check = self._execute_grasping_twin_js_2(grasping_pose_world[i], idx=key)
                if check:
                    break

            if check:
                break

        if not check:
            cprint("G=================== Grasping task failed ===================", "red")
            return False

        cprint("G=================== 5. Successfully completed the grasping task ===================", "green")

        # ---- Placement phase ----
        if container.lower() == "person":
            cprint("H=================== Handover mode detected: delivering to person ===================", "cyan")
            pose_1st = self.config.get_pose("get_ready_to_handover_1st")
            pose_2nd = self.config.get_pose("get_ready_to_handover_2nd")
            if self.config.get_pose("handover_pose") is None:
                cprint("H=================== Handover task failed: handover_pose not found ===================", "red")
                return False
            if pose_1st is not None:
                self.control_arm(pose_type="get_ready_to_handover_1st", speed=15)
            if pose_2nd is not None:
                self.control_arm(pose_type="get_ready_to_handover_2nd", speed=15)
            self.control_arm(pose_type="handover_pose", speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            self.control_arm(pose_type="grasp1", speed=30)
            cprint("H=================== 5. Successfully completed the handover task ===================", "green")
            return True

        if container.lower() in ["trash", "垃圾桶", "garbage", "bin"]:
            cprint("T=================== Trash mode detected: throwing to trash ===================", "cyan")
            throw_pose = self.config.get_pose("throw_to_trash_pose")
            if throw_pose is None:
                cprint("T=================== Trash task failed: throw_to_trash_pose not found ===================", "red")
                return False
            joint_angles = [throw_pose[f"J{i}"] for i in range(1, 8)]
            self.control_arm(trajectory=np.array([joint_angles]), speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            self.control_arm(pose_type="grasp1", speed=30)
            cprint("T=================== 5. Successfully completed the trash task ===================", "green")
            return True

        if container.lower() in ["desk", "桌子", "table"]:
            cprint("D=================== Desk placement mode detected: placing on desk ===================", "cyan")
            selected = random.choice(["desk_pose_1", "desk_pose_2", "desk_pose_3"])
            desk_pose = self.config.get_pose(selected)
            if desk_pose is None:
                cprint(f"D=================== Desk task failed: {selected} not found ===================", "red")
                return False
            joint_angles = [desk_pose[f"J{i}"] for i in range(1, 8)]
            self.control_arm(trajectory=np.array([joint_angles]), speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            self.control_arm(pose_type="grasp1", speed=30)
            cprint("D=================== 5. Successfully completed the desk placement task ===================", "green")
            return True

        # Normal vision-based placement
        for key, value in self.config.default_traj_js.items():
            if "grasp" not in key:
                continue
            self.control_arm(pose_type=key, speed=30)
            self.rgb, self.depth = self.get_camera_obs()
            cprint(f"P=================== 3. Save current rgb and depth observations: {self.save_path} ===================", "cyan")
            self.save_current_transformation()
            placing_pos_world = self._get_placing_position(class_name=container, image=self.rgb)
            cprint("P=================== 4. Generate the placing pose ===================", "cyan")

            if not len(placing_pos_world):
                continue
            check = self._execute_placement(placing_pos_world)
            if check:
                break

        self.control_arm(pose_type="grasp1", speed=30)
        cprint("P=================== 5. Successfully completed the placement task ===================", "green")
        return True

    # ------------------------------------------------------------------
    # Grasp helpers (from planner.py)
    # ------------------------------------------------------------------
    def _filtering_pose(self, anygrasp_pose, class_name="", image=None):
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
                class_names = [cls for cls in class_name.split(',')]
                for i in sorted_indices:
                    g_pose = grasp_pose_cam[i]
                    final_grasps.append(g_pose)
                cprint(
                    f"*********** Class name {class_names} ****************** "
                    f"Grasp pose number: {len(final_grasps)} ******************", "red")
                ans = True
            else:
                cprint(f"Found objects ({class_name}) but NO grasp points inside them.", "yellow")
        else:
            cprint(f"No object detected for class: {class_name}", "yellow")

        return final_grasps if ans else []

    def _transform_anygrasp_pose(self, anygrasp_pose, _visualization=True, return_labels=False):
        try:
            eval_score = {}
            for id, data in enumerate(anygrasp_pose[:]):
                grasp_translation = data['trans']
                grasp_rotation_matrix = data['rotation_matrix']

                grasp_transformation_matrix = np.eye(4, 4)
                grasp_transformation_matrix[:3, :3] = grasp_rotation_matrix
                grasp_transformation_matrix[:3, 3] = grasp_translation

                self_pose_matrix = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                self_pose_matrix = self_rotation_np(self_pose_matrix)

                transformed_pose_camera = grasp_transformation_matrix @ self_pose_matrix
                transformed_pose_world = self.T_base_to_cam @ transformed_pose_camera
                transformed_pose_world = self._transform_x_axis(transformed_pose_world)

                eval_score[id] = {}
                eval_score[id]["score"] = data["score"]
                eval_score[id]["transformed_pose_world"] = transformed_pose_world
                eval_score[id]["original_pose"] = anygrasp_pose[id]
                if return_labels:
                    eval_score[id]["label"] = data["label"]

            final_grasp_pose_data = sorted(eval_score.items(), key=lambda x: x[1]["score"], reverse=True)
            return [final_grasp_pose_data[i][1]["transformed_pose_world"] for i in range(len(final_grasp_pose_data))]
        except Exception:
            return []

    def _transform_x_axis(self, transformed_pose_world):
        r, p, y = R.from_matrix(transformed_pose_world[:3, :3]).as_euler('xyz', degrees=False)
        x_axis_rotated = rpy_to_vector(r, p, y, axis=[1, 0, 0])
        y_axis_world = np.array([0, 1, 0])
        cos_theta = np.dot(x_axis_rotated, y_axis_world) / (
            np.linalg.norm(x_axis_rotated) * np.linalg.norm(y_axis_world)
        )
        if cos_theta > 0:
            transformed_pose_world = transformed_pose_world @ np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        return transformed_pose_world

    def _execute_grasping_twin_js_2(self, grasping_pose_world_hand, idx=None):
        """Exact replica of planner.py execute_grasping_twin_js_2."""
        self.control_hand(cmd_type="open")
        basic_mat = np.eye(4)
        basic_mat[2, 3] -= 0.02
        preparasion_grasping_pos = grasping_pose_world_hand @ basic_mat
        preparasion_grasping_pos[2, 3] += 0.02
        preparasion_grasping_pos = preparasion_grasping_pos @ self.T_hand_effector_to_arm_endlink
        preparasion_grasping_pos_position = preparasion_grasping_pos[:3, 3]
        preparasion_grasping_pos_orientation = R.from_matrix(preparasion_grasping_pos[:3, :3]).as_quat()

        basic_mat = np.eye(4)
        execution_grasping_pos = grasping_pose_world_hand @ basic_mat
        print("=-============================", execution_grasping_pos[2, 3])
        execution_grasping_pos[2, 3] = max(execution_grasping_pos[2, 3], 0.042)
        execution_grasping_pos = execution_grasping_pos @ self.T_hand_effector_to_arm_endlink
        execution_grasping_pos_position = execution_grasping_pos[:3, 3]
        execution_grasping_pos_orientation = R.from_matrix(execution_grasping_pos[:3, :3]).as_quat()

        default_traj_js_rad = [data / 180 * np.pi for data in self.config.default_traj_js[idx].values()]
        cnfg = {
            "target_pose": [
                [preparasion_grasping_pos_position[0], preparasion_grasping_pos_position[1], preparasion_grasping_pos_position[2],
                 preparasion_grasping_pos_orientation[0], preparasion_grasping_pos_orientation[1], preparasion_grasping_pos_orientation[2], preparasion_grasping_pos_orientation[3]],
                [execution_grasping_pos_position[0], execution_grasping_pos_position[1], execution_grasping_pos_position[2],
                 execution_grasping_pos_orientation[0], execution_grasping_pos_orientation[1], execution_grasping_pos_orientation[2], execution_grasping_pos_orientation[3]],
            ],
            "current_js": default_traj_js_rad,
            "struct": "left_arm",
        }
        rsp = self.send_cmd_twin(self.twin, {"srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg})
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
            cprint("********************* The preparasion pose is not reachable !! *********************")
            return False

    # ------------------------------------------------------------------
    # Placement helpers (from planner.py)
    # ------------------------------------------------------------------
    def _get_placing_position(self, class_name=None, image=None):
        detections = self.perception.detect_objects(image, [class_name], conf=0.25)
        try:
            if len(detections):
                det = detections[0][:4]
                if det[1] >= 400 and det[3] <= 480 and len(detections) > 1:
                    det = detections[1][:4]
                x1, y1, x2, y2 = [int(coord) for coord in det]

                H, W = self.depth.shape
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(W, x2)
                y2 = min(H, y2)

                depth_sub_image_mm = self.depth[y1:y2, x1:x2]
                valid_depths_mm = depth_sub_image_mm[depth_sub_image_mm > 0]

                if len(valid_depths_mm) > 0:
                    mean_depth_mm = np.median(valid_depths_mm)
                else:
                    print("Warning: No valid depth values found in the bounding box.")
                    return []

                mean_depth_m = mean_depth_mm * 1e-3

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                center_cam_point = pixel_to_camera_point2(np.array([center_x, center_y]).reshape(-1, 2), mean_depth_m)
                center_cam_point = center_cam_point.flatten()
                placing_pos_world = self._transform_pose_to_world(center_cam_point)
                return placing_pos_world
        except Exception:
            pass
        return []

    def _transform_pose_to_world(self, pose_cam_point):
        placing_translation = pose_cam_point.flatten()
        T_cam_point = np.eye(4, 4)
        T_cam_point[:3, :3] = R.from_quat([-0.210, 0.016, -0.056, 0.976]).as_matrix()
        T_cam_point[:3, 3] = placing_translation
        T_world_point = self.T_base_to_cam @ T_cam_point
        T_world_pose = T_world_point.copy()
        return T_world_pose

    def _execute_placement(self, placement_pos_world):
        placement_pos_world[2, 3] += 0.15
        placement_pos_arm = placement_pos_world @ self.T_hand_effector_to_arm_endlink
        pos = placement_pos_arm[:3, 3]
        orn = R.from_matrix(placement_pos_arm[:3, :3]).as_quat()

        default_js_rad = [v / 180 * np.pi for v in self.config.default_traj_js["grasp1"].values()]
        cnfg = {
            "target_pose": [[pos[0], pos[1], pos[2], orn[0], orn[1], orn[2], orn[3]]],
            "current_js": default_js_rad,
            "struct": "left_arm",
        }
        rsp = self.send_cmd_twin(self.twin, {"srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg})
        if rsp["value"]:
            traj = np.array(copy.deepcopy(rsp["info"]["trajectory"])) / np.pi * 180
            self.control_arm(trajectory=traj, speed=20)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            return True
        return False
