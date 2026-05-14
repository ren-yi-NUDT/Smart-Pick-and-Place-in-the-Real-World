import copy
import random
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core.transforms import graspcam2pixel, self_rotation_np, rpy_to_vector, pixel_to_camera_point2
from core.world_memory import setup_world_memory
from core.world_model_critic import WorldModelCritic


def _mode_from_container(container):
    """Determine world memory mode from container string."""
    if container is None:
        return "normal_place"
    c = container.lower()
    if c == "person":
        return "handover"
    if c in ("trash", "垃圾桶", "garbage", "bin"):
        return "trash"
    if c in ("desk", "桌子", "table"):
        return "desk_place"
    return "normal_place"


@register_skill("pick_and_place")
class PickAndPlaceSkill(Skill):
    """Main pick-and-place pipeline: detect → grasp → place, with full world
    memory + critic evaluation at every stage."""

    def run(self, **kwargs):
        if kwargs.get("object") and kwargs.get("container"):
            json_data = kwargs
        else:
            json_data = self.json_parser.get_command()
        if json_data is None:
            cprint("未收到有效的JSON输入", "red")
            return {"success": False, "stage": "input", "reason": "no_json"}

        cprint(f"=================== 1. Get JSON input: {json_data} ===================", "cyan")

        obj, container = json_data.get("object"), json_data.get("container")
        if obj is None or container is None:
            cprint("JSON输入缺少必需字段 (object 或 container)", "red")
            return {"success": False, "stage": "input", "reason": "missing_object_or_container"}

        cprint(f"=================== 2. Parse input: Grasp {obj} and place it in/on/to {container} ===================", "cyan")

        # ------------------------------------------------------------------
        # Setup world memory + critic
        # ------------------------------------------------------------------
        mode = _mode_from_container(container)
        memory = setup_world_memory({"object": obj, "container": container})
        memory.data["mode"] = mode
        critic = WorldModelCritic(memory)

        memory.record_action({"type": "parse_input", "input": {"object": obj, "container": container}})

        # ------------------------------------------------------------------
        # 3. Critic: before_grasp
        # ------------------------------------------------------------------
        before_grasp = critic.before_grasp()
        if not before_grasp.get("approved", False):
            cprint("C=================== Critic blocked grasp ===================", "red")
            return {
                "success": False, "stage": "before_grasp",
                "object": obj, "container": container,
                "critic": before_grasp,
                "memory_id": memory.memory_id,
            }

        cprint("C=================== Critic approved grasp ===================", "green")

        # ------------------------------------------------------------------
        # 4. Grasp phase (with retries)
        # ------------------------------------------------------------------
        MAX_GRASP_RETRIES = 2
        grasp_result = {"success": False, "hand_closed": False, "finger_deviation": 0.0}
        grasp_attempts = 0

        for grasp_retry in range(MAX_GRASP_RETRIES):
            if grasp_retry > 0:
                cprint(f"G=================== Grasp retry {grasp_retry}/{MAX_GRASP_RETRIES-1} ===================", "yellow")
                self.safe_release(safe_pose="grasp1")
                time.sleep(0.5)

            self.control_arm(pose_type="grasp1", speed=30)
            self.control_hand(cmd_type="close")

            for key in self.config.default_traj_js:
                if "grasp" not in key:
                    continue
                self.control_arm(pose_type=key, speed=30)
                self.rgb, self.depth = self.get_camera_obs()
                cprint(f"G=================== 3. Save current rgb and depth observations: {self.save_path} ===================", "cyan")

                anygrasp_pose = self.perception.detect_grasps(self.rgb, self.depth)
                cprint(f"G=================== 4. Generate the grasping pose ===================", "cyan")
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
                    grasp_attempts += 1
                    cprint(f"=================== Checking pose: {i+1} / {len(grasping_pose_world)} ===================", "yellow")
                    grasp_result = self._execute_grasping_twin_js_2(grasping_pose_world[i], idx=key)
                    if grasp_result.get("success"):
                        break
                if grasp_result.get("success"):
                    break
            if grasp_result.get("success"):
                break

        # ------------------------------------------------------------------
        # 5. Critic: verify_grasp
        # ------------------------------------------------------------------
        grasp_result["object"] = obj
        memory.record_action({"type": "grasp", "result": grasp_result, "attempts": grasp_attempts})
        grasp_verification = critic.verify_grasp(grasp_result)

        if not grasp_result.get("success") or not grasp_verification.get("success", False):
            cprint("G=================== Grasping task failed ===================", "red")
            self.safe_release(safe_pose="grasp1")
            return {
                "success": False, "stage": "grasp_failed",
                "object": obj, "container": container,
                "grasp_result": grasp_result,
                "grasp_verification": grasp_verification,
                "grasp_attempts": grasp_attempts,
                "memory_id": memory.memory_id,
            }

        cprint("G=================== 5. Successfully completed the grasping task ===================", "green")

        # ------------------------------------------------------------------
        # 6. Critic: before_destination
        # ------------------------------------------------------------------
        before_dest = critic.before_destination_action()
        if not before_dest.get("approved", False):
            cprint("C=================== Critic blocked destination action ===================", "red")
            return {
                "success": False, "stage": "before_destination",
                "object": obj, "container": container,
                "critic": before_dest,
                "memory_id": memory.memory_id,
            }

        cprint(f"C=================== Critic approved destination: {before_dest.get('next_action')} ===================", "green")

        # ------------------------------------------------------------------
        # 7. Placement phase (with retries)
        # ------------------------------------------------------------------
        MAX_PLACE_RETRIES = 2
        dest_result = {"success": False, "type": "unknown", "container": container, "hand_opened": False}
        dest_type = "unknown"
        dest_attempts = 0

        if container.lower() == "person":
            dest_type = "handover"
            cprint("H=================== Handover mode detected ===================", "cyan")
            pose_1st = self.config.get_pose("get_ready_to_handover_1st")
            pose_2nd = self.config.get_pose("get_ready_to_handover_2nd")
            handover_pose = self.config.get_pose("handover_pose")

            for dest_retry in range(MAX_PLACE_RETRIES):
                if dest_retry > 0:
                    cprint(f"H=================== Handover retry {dest_retry}/{MAX_PLACE_RETRIES-1} ===================", "yellow")
                    time.sleep(0.5)

                if handover_pose is None:
                    cprint("H=================== Handover task failed: handover_pose not found ===================", "red")
                    break
                if pose_1st is not None:
                    self.control_arm(pose_type="get_ready_to_handover_1st", speed=15)
                if pose_2nd is not None:
                    self.control_arm(pose_type="get_ready_to_handover_2nd", speed=15)
                self.control_arm(pose_type="handover_pose", speed=15)
                time.sleep(0.5)
                self.control_hand(cmd_type="open")
                time.sleep(0.3)
                hand_opened = self.hand.is_fully_open()
                cprint(f"H=================== Hand fully open: {hand_opened} ===================",
                       "green" if hand_opened else "yellow")
                time.sleep(0.7)
                self.control_arm(pose_type="grasp1", speed=30)
                dest_attempts += 1
                dest_result = {"success": True, "type": dest_type, "container": container, "hand_opened": hand_opened}
                if hand_opened:
                    break
                else:
                    cprint("H=================== Hand did not open, retrying... ===================", "yellow")

        elif container.lower() in ["trash", "垃圾桶", "garbage", "bin"]:
            dest_type = "trash"
            cprint("T=================== Trash mode detected ===================", "cyan")
            throw_pose = self.config.get_pose("throw_to_trash_pose")

            for dest_retry in range(MAX_PLACE_RETRIES):
                if dest_retry > 0:
                    cprint(f"T=================== Trash retry {dest_retry}/{MAX_PLACE_RETRIES-1} ===================", "yellow")
                    time.sleep(0.5)

                if throw_pose is None:
                    cprint("T=================== Trash task failed: throw_to_trash_pose not found ===================", "red")
                    break
                joint_angles = [throw_pose[f"J{i}"] for i in range(1, 8)]
                self.control_arm(trajectory=np.array([joint_angles]), speed=15)
                time.sleep(0.5)
                self.control_hand(cmd_type="open")
                time.sleep(0.3)
                hand_opened = self.hand.is_fully_open()
                cprint(f"T=================== Hand fully open: {hand_opened} ===================",
                       "green" if hand_opened else "yellow")
                time.sleep(0.7)
                self.control_arm(pose_type="grasp1", speed=30)
                dest_attempts += 1
                dest_result = {"success": True, "type": dest_type, "container": container, "hand_opened": hand_opened}
                if hand_opened:
                    break
                else:
                    cprint("T=================== Hand did not open, retrying... ===================", "yellow")

        elif container.lower() in ["desk", "桌子", "table"]:
            dest_type = "desk_place"
            cprint("D=================== Desk placement mode detected ===================", "cyan")

            for dest_retry in range(MAX_PLACE_RETRIES):
                if dest_retry > 0:
                    cprint(f"D=================== Desk placement retry {dest_retry}/{MAX_PLACE_RETRIES-1} ===================", "yellow")
                    time.sleep(0.5)

                selected = random.choice(["desk_pose_1", "desk_pose_2", "desk_pose_3"])
                desk_pose = self.config.get_pose(selected)
                if desk_pose is None:
                    cprint(f"D=================== Desk task failed: {selected} not found ===================", "red")
                    break
                joint_angles = [desk_pose[f"J{i}"] for i in range(1, 8)]
                self.control_arm(trajectory=np.array([joint_angles]), speed=15)
                time.sleep(0.5)
                self.control_hand(cmd_type="open")
                time.sleep(0.3)
                hand_opened = self.hand.is_fully_open()
                cprint(f"D=================== Hand fully open: {hand_opened} ===================",
                       "green" if hand_opened else "yellow")
                time.sleep(0.7)
                self.control_arm(pose_type="grasp1", speed=30)
                dest_attempts += 1
                dest_result = {"success": True, "type": dest_type, "container": container, "hand_opened": hand_opened}
                if hand_opened:
                    break
                else:
                    cprint("D=================== Hand did not open, retrying... ===================", "yellow")

        else:
            dest_type = "place"
            cprint(f"P=================== Normal placement: {container} ===================", "cyan")

            for dest_retry in range(MAX_PLACE_RETRIES):
                if dest_retry > 0:
                    cprint(f"P=================== Placement retry {dest_retry}/{MAX_PLACE_RETRIES-1} ===================", "yellow")
                    time.sleep(0.5)

                for key in self.config.default_traj_js:
                    if "grasp" not in key:
                        continue
                    self.control_arm(pose_type=key, speed=30)
                    self.rgb, self.depth = self.get_camera_obs()
                    self.save_current_transformation()
                    placing_pos_world = self._get_placing_position(class_name=container, image=self.rgb)
                    if not len(placing_pos_world):
                        continue
                    dest_attempts += 1
                    dest_result = self._execute_placement(placing_pos_world)
                    if dest_result.get("success"):
                        break
                if dest_result.get("success"):
                    break
                else:
                    cprint(f"P=================== Container not found from any observation pose, retry {dest_retry+1}/{MAX_PLACE_RETRIES} ===================", "yellow")

            self.control_arm(pose_type="grasp1", speed=30)

        # ------------------------------------------------------------------
        # 8. Critic: verify_destination + verify_goal
        # ------------------------------------------------------------------
        dest_result["attempts"] = dest_attempts
        memory.record_action({"type": dest_type, "result": dest_result})
        dest_verification = critic.verify_destination_action(dest_result)

        if not dest_result.get("success") or not dest_verification.get("success", False):
            cprint(f"{dest_type.upper()}=================== {dest_type} task failed ===================", "red")
            self.safe_release(safe_pose="grasp1")
            return {
                "success": False, "stage": "destination_failed",
                "object": obj, "container": container,
                "destination_type": dest_type,
                "destination_result": dest_result,
                "destination_verification": dest_verification,
                "grasp_result": grasp_result,
                "grasp_verification": grasp_verification,
                "grasp_attempts": grasp_attempts,
                "dest_attempts": dest_attempts,
                "memory_id": memory.memory_id,
            }

        cprint(f"{dest_type.upper()}=================== 5. Successfully completed the {dest_type} task ===================", "green")

        # Final goal verification
        total_recoveries = max(0, grasp_attempts - 1) + max(0, dest_attempts - 1)
        goal_payload = {
            "object": obj, "container": container, "mode": mode,
            "grasp_result": grasp_result, "grasp_verification": grasp_verification,
            "destination_result": dest_result, "destination_verification": dest_verification,
            "resolved_failures": total_recoveries,
        }
        goal = critic.verify_goal(goal_payload)
        success = bool(goal.get("success", False))

        result = {
            "success": success, "stage": "completed" if success else "goal_not_verified",
            "object": obj, "container": container, "mode": mode,
            "grasp_result": grasp_result, "grasp_verification": grasp_verification,
            "destination_type": dest_type, "destination_result": dest_result,
            "destination_verification": dest_verification,
            "grasp_attempts": grasp_attempts, "dest_attempts": dest_attempts,
            "recoveries": total_recoveries,
            "goal": goal, "memory_id": memory.memory_id,
        }

        if success:
            cprint("P=================== Successfully completed the full task ===================", "green")
        else:
            cprint("P=================== Task finished but goal not verified ===================", "red")

        return result

    # ------------------------------------------------------------------
    # Grasp helpers (from planner.py)
    # ------------------------------------------------------------------
    def _filtering_pose(self, anygrasp_pose, class_name="", image=None):
        detections = self.perception.detect_objects(image, class_name, conf=0.2)
        grasp_points, grasp_pose_cam = graspcam2pixel(anygrasp_pose)
        valid_indices = set()
        final_grasps = []
        ans = False
        if len(detections):
            det = detections[0][:4]
            x1, y1, x2, y2 = det
            for i, grasp_p in enumerate(grasp_points):
                if grasp_p[0] > x1 - 20 and grasp_p[0] < x2 + 20 and \
                   grasp_p[1] > y1 - 20 and grasp_p[1] < y2 + 20:
                    valid_indices.add(i)
            if len(valid_indices):
                sorted_indices = sorted(list(valid_indices))
                for i in sorted_indices:
                    final_grasps.append(grasp_pose_cam[i])
                cprint(f"*********** Grasp pose number: {len(final_grasps)} ******************", "red")
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
                eval_score[id] = {"score": data["score"], "transformed_pose_world": transformed_pose_world}
            final = sorted(eval_score.items(), key=lambda x: x[1]["score"], reverse=True)
            return [final[i][1]["transformed_pose_world"] for i in range(len(final))]
        except Exception:
            return []

    def _transform_x_axis(self, transformed_pose_world):
        r, p, y = R.from_matrix(transformed_pose_world[:3, :3]).as_euler('xyz', degrees=False)
        x_axis_rotated = rpy_to_vector(r, p, y, axis=[1, 0, 0])
        y_axis_world = np.array([0, 1, 0])
        cos_theta = np.dot(x_axis_rotated, y_axis_world) / (
            np.linalg.norm(x_axis_rotated) * np.linalg.norm(y_axis_world))
        if cos_theta > 0:
            transformed_pose_world = transformed_pose_world @ np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        return transformed_pose_world

    def _execute_grasping_twin_js_2(self, grasping_pose_world_hand, idx=None):
        self.control_hand(cmd_type="open")
        basic_mat = np.eye(4)
        basic_mat[2, 3] -= 0.02
        preparasion_grasping_pos = grasping_pose_world_hand @ basic_mat
        preparasion_grasping_pos[2, 3] += 0.02
        preparasion_grasping_pos = preparasion_grasping_pos @ self.T_hand_effector_to_arm_endlink
        prep_pos = preparasion_grasping_pos[:3, 3]
        prep_orn = R.from_matrix(preparasion_grasping_pos[:3, :3]).as_quat()
        basic_mat = np.eye(4)
        execution_grasping_pos = grasping_pose_world_hand @ basic_mat
        execution_grasping_pos[2, 3] = max(execution_grasping_pos[2, 3], 0.042)
        execution_grasping_pos = execution_grasping_pos @ self.T_hand_effector_to_arm_endlink
        exec_pos = execution_grasping_pos[:3, 3]
        exec_orn = R.from_matrix(execution_grasping_pos[:3, :3]).as_quat()
        default_traj_js_rad = [data / 180 * np.pi for data in self.config.default_traj_js[idx].values()]
        cnfg = {
            "target_pose": [
                [prep_pos[0], prep_pos[1], prep_pos[2], prep_orn[0], prep_orn[1], prep_orn[2], prep_orn[3]],
                [exec_pos[0], exec_pos[1], exec_pos[2], exec_orn[0], exec_orn[1], exec_orn[2], exec_orn[3]],
            ],
            "current_js": default_traj_js_rad,
            "struct": "left_arm",
        }
        rsp = self.send_cmd_twin(self.twin, {"srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg})
        if rsp["value"]:
            trajectory = np.array(copy.deepcopy(rsp["info"]["trajectory"])) / np.pi * 180
            self.control_arm(trajectory=trajectory, speed=20)
            cprint("=============== Reach grasping pose =============")
            self.control_hand(cmd_type="close")
            cprint("=============== Close hand =============")
            time.sleep(0.5)

            # Real verification: read hand motor state after closing
            hand_state = self.control_hand(cmd_type="get_state")
            finger_deviation = self.hand.get_finger_deviation()
            cprint(f"=============== Finger deviation after close: {finger_deviation:.1f} =============",
                   "green" if finger_deviation > 5 else "yellow")

            self.control_arm(pose_type=idx, speed=30)
            cprint("=============== Reach post grasping pose =============")

            return {
                "success": True,
                "hand_closed": True,
                "finger_deviation": finger_deviation,
            }
        else:
            cprint("********************* The preparasion pose is not reachable !! *********************")
            return {"success": False, "hand_closed": False, "finger_deviation": 0.0}

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
                x1, y1, x2, y2 = [int(c) for c in det]
                H, W = self.depth.shape
                x1, y1, x2, y2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)
                depth_sub = self.depth[y1:y2, x1:x2]
                valid = depth_sub[depth_sub > 0]
                if len(valid) == 0:
                    return []
                mean_depth_m = np.median(valid) * 1e-3
                center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
                center_cam = pixel_to_camera_point2(np.array([center_x, center_y]).reshape(-1, 2), mean_depth_m).flatten()
                return self._transform_pose_to_world(center_cam)
        except Exception:
            pass
        return []

    def _transform_pose_to_world(self, pose_cam_point):
        T_cam_point = np.eye(4, 4)
        T_cam_point[:3, :3] = R.from_quat([-0.210, 0.016, -0.056, 0.976]).as_matrix()
        T_cam_point[:3, 3] = pose_cam_point.flatten()
        return (self.T_base_to_cam @ T_cam_point).copy()

    def _execute_placement(self, placement_pos_world):
        placement_pos_world[2, 3] += 0.15
        pos = (placement_pos_world @ self.T_hand_effector_to_arm_endlink)[:3, 3]
        orn = R.from_matrix((placement_pos_world @ self.T_hand_effector_to_arm_endlink)[:3, :3]).as_quat()
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
            time.sleep(0.3)
            hand_opened = self.hand.is_fully_open()
            cprint(f"P=================== Hand fully open: {hand_opened} ===================",
                   "green" if hand_opened else "yellow")
            time.sleep(0.7)
            return {"success": True, "hand_opened": hand_opened, "type": "place"}
        return {"success": False, "hand_opened": False, "type": "place"}
