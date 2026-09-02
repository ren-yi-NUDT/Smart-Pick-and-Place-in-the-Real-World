import copy
import random
import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core.transforms import graspcam2pixel, self_rotation_np, pixel_to_camera_point2


@register_skill("pick_and_place")
class PickAndPlaceSkill(Skill):
    """Main pick-and-place pipeline: detect → grasp → place."""

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
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
        side = json_data.get("side", "left")
        location = json_data.get("location", "desk_front")
        if obj is None or container is None:
            cprint("JSON输入缺少必需字段 (object 或 container)", "red")
            return False

        cprint(f"=================== 2. Parse input: [{side}] Grasp {obj} and place it in the {container} ===================", "cyan")

        # ---- Grasp phase ----
        if obj.lower() in ["user", "user_hand", "hand"]:
            check = self._receive_from_user(side)
        else:
            check = self._visual_grasp_phase(obj, side=side, location=location)

        if not check:
            return False

        cprint("G=================== 5. Successfully completed the grasping task ===================", "green")

        # ---- Placement phase ----
        if container.lower() == "person":
            return self._place_handover_person(side)

        if container.lower() in ["trash", "垃圾桶", "garbage", "bin"]:
            return self._place_trash(side)

        if container.lower() in ["desk", "桌子", "table"]:
            if side == "right" and not self._handover_right_to_left():
                return False
            return self._place_desk()

        # Drawer / cabinet — narrow receptacle, only right gripper fits.
        # Either arm can grasp; left arm must hand over to right before placing.
        if self._is_right_fixed_placement(container):
            if side == "right":
                return self._place_predefined(container)
            if not self._delegate_to_left_arm(container):
                return False
            return self._place_predefined(container)

        # Visual placement (bowls, plates, etc.) — uses left-arm camera.
        # Right arm must hand over to left arm first.
        if side == "right":
            if not self._handover_right_to_left():
                return False
            return self._place_visual(container)

        return self._place_visual(container)

    # ------------------------------------------------------------------
    # Placement routing helpers
    # ------------------------------------------------------------------
    _DRAWER_KEYWORDS = ("drawer", "cabinet", "抽屉", "柜")

    def _is_right_fixed_placement(self, container):
        """Check if container is a right-arm fixed placement target (drawer/cabinet).
        All drawer-like containers route to the canonical `drawer_1_placement` pose.
        """
        c = container.lower()
        return any(k in c for k in self._DRAWER_KEYWORDS)

    def _place_predefined(self, pose_name):
        """Right arm moves to a predefined pose and releases the object.
        All drawer/cabinet keywords normalize to `drawer_1_placement`.
        """
        from core.arm import ArmClient
        from core.gripper import GripperClient

        right_cfg = self.config.get_arm_config("right")
        if any(k in pose_name.lower() for k in self._DRAWER_KEYWORDS):
            target_pose_name = "drawer_1_placement"
        else:
            target_pose_name = pose_name

        cprint(f"P=================== Right arm fixed placement: {pose_name} → {target_pose_name} ===================", "cyan")

        right_arm = self.arm_for("right")
        right_gripper = self.gripper_for("right")

        pose = right_cfg.get(target_pose_name)
        if pose is None:
            cprint(f"P=================== Pose '{target_pose_name}' not found in right arm config ===================", "red")
            return False

        right_arm.move_to_named_pose(pose, speed=15)
        right_gripper.open()
        time.sleep(1)
        right_arm.move_to_named_pose(right_cfg["home"], speed=30)
        cprint(f"P=================== Fixed placement to {target_pose_name} done ===================", "green")
        return True

    def _delegate_to_left_arm(self, container, return_home=True):
        """Replay the latest recorded left→right dual-arm handover trajectory.

        The latest recording contains the synchronized arm motion and the
        timed gripper events (right close, left open, then both arms home).
        ``container`` is retained for callers that use the older API.
        """
        cprint(
            f"D=================== Two-arm handover (latest recording, container={container}) ===================",
            "cyan",
        )
        if not return_home:
            cprint(
                "D=================== Latest recording includes the final home motion ===================",
                "yellow",
            )
        try:
            from tools.play_dual_handover import play

            ok = play(
                "dual_handover_timed_20260826_v2",
                speed=0.9,
                require_confirmation=False,
                direction="left_to_right",
            )
            if ok:
                cprint(
                    "D=================== Latest handover done (object now in right gripper at home) ===================",
                    "green",
                )
            return bool(ok)
        except Exception as exc:
            cprint(f"D=================== Latest handover failed: {exc} ===================", "red")
            return False

    def _handover_right_to_left(self):
        """Right→left handover: same recorded trajectory as left→right,
        gripper event roles swapped via ``direction`` (left closes to
        receive, right opens to release).

        Used when right arm grasped the object and left arm needs to place it
        (side=right + visual container, or side=right + person).
        """
        cprint("H=================== Right→left handover (recorded replay) ===================", "cyan")
        try:
            from tools.play_dual_handover import play

            ok = play(
                "dual_handover_timed_20260826_v2",
                speed=0.9,
                require_confirmation=False,
                direction="right_to_left",
            )
            if ok:
                cprint(
                    "H=================== Handover done (object now in left gripper at home) ===================",
                    "green",
                )
            return bool(ok)
        except Exception as exc:
            cprint(f"H=================== Right→left handover failed: {exc} ===================", "red")
            return False

    # ------------------------------------------------------------------
    def _ready_grasp_hand(self) -> None:
        """Pre-grasp end-effector prep: open the Robotiq 85 gripper."""
        self.control_hand(cmd_type="open")

    # ------------------------------------------------------------------
    def _visual_grasp_phase(self, obj, side="left", location="desk_front"):
        """Unified visual grasp for both arms."""
        return self.visual_grasp(obj, side=side, location=location)

    def _visual_grasp_right(self, obj, location="desk_front"):
        """Compatibility wrapper retained for callers of the old right path."""
        return self.visual_grasp(obj, side="right", location=location)

    def _save_right_arm_transforms(self):
        """Compatibility wrapper for the side-aware TF cache."""
        self.save_current_transformation("right")

    def _receive_from_user(self, side="left"):
        """Receive object from user at handover pose."""
        cprint("R=================== Receive from user mode ===================", "cyan")

        # Move to handover pose via waypoints
        if side == "left":
            pose_1st = self.config.get_pose("get_ready_to_handover_1st")
            pose_2nd = self.config.get_pose("get_ready_to_handover_2nd")
            if pose_1st is not None:
                self.control_arm(pose_type="get_ready_to_handover_1st", speed=15)
            if pose_2nd is not None:
                self.control_arm(pose_type="get_ready_to_handover_2nd", speed=15)
        self.control_arm(pose_type="handover_pose", speed=15)

        # Open hand and wait for user
        self.control_hand(cmd_type="open")
        cprint("R=================== Waiting for user to place object... ===================", "cyan")
        time.sleep(1)
        time.sleep(3)

        # Close hand to grasp
        self.control_hand(cmd_type="close")
        time.sleep(0.5)
        cprint("R=================== Received object from user ===================", "green")

        # Return to safe pose
        if side == "left":
            pose_2nd = self.config.get_pose("get_ready_to_handover_2nd")
            pose_1st = self.config.get_pose("get_ready_to_handover_1st")
            if pose_2nd is not None:
                self.control_arm(pose_type="get_ready_to_handover_2nd", speed=15)
            if pose_1st is not None:
                self.control_arm(pose_type="get_ready_to_handover_1st", speed=15)
        self.control_arm(pose_type="home", speed=30)
        return True

    # ------------------------------------------------------------------
    # Placement phase variants
    # ------------------------------------------------------------------
    def _place_handover_person(self, side="left"):
        """Deliver object to person."""
        cprint("H=================== Handover mode detected: delivering to person ===================", "cyan")

        if side == "left":
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
            time.sleep(2)
            cprint("H=================== Retracing path back ===================", "cyan")
            if pose_2nd is not None:
                self.control_arm(pose_type="get_ready_to_handover_2nd", speed=15)
            if pose_1st is not None:
                self.control_arm(pose_type="get_ready_to_handover_1st", speed=15)
            self.control_arm(pose_type="home", speed=30)
            cprint("H=================== 5. Successfully completed the handover task ===================", "green")
            return True
        else:
            # Right arm → left arm handover → left arm delivers to person
            cprint("H=================== Dual-arm handover: right → left → person ===================", "cyan")

            # Phase 1: Validated 4-step right→left handover (ends with both arms at home, object in left gripper)
            if not self._handover_right_to_left():
                cprint("H=================== Handover failed ===================", "red")
                return False

            # Phase 2: Left arm delivers object to person via preset waypoints
            cprint("H=================== Left arm → handover to person ===================", "cyan")
            pose_1st = self.config.get_pose("get_ready_to_handover_1st")
            pose_2nd = self.config.get_pose("get_ready_to_handover_2nd")
            if pose_1st is not None:
                self.control_arm(pose_type="get_ready_to_handover_1st", speed=15)
            if pose_2nd is not None:
                self.control_arm(pose_type="get_ready_to_handover_2nd", speed=15)
            self.control_arm(pose_type="handover_pose", speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(2)
            if pose_2nd is not None:
                self.control_arm(pose_type="get_ready_to_handover_2nd", speed=15)
            if pose_1st is not None:
                self.control_arm(pose_type="get_ready_to_handover_1st", speed=15)
            self.control_arm(pose_type="home", speed=30)

            cprint("H=================== 5. Successfully completed the handover task ===================", "green")
            return True

    def _place_trash(self, side="left"):
        """Throw object to trash. Left arm transfers to right arm first."""
        cprint("T=================== Trash mode detected ===================", "cyan")

        if side == "left":
            return self._dual_arm_trash()
        else:
            return self._single_arm_trash()

    def _place_desk(self):
        """Place object on desk."""
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
        self.control_arm(pose_type="home", speed=30)
        cprint("D=================== 5. Successfully completed the desk placement task ===================", "green")
        return True

    def _place_visual(self, container):
        """Vision-based placement into detected container."""
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
            check = self._execute_placement(placing_pos_world, initial_js_key=key)
            if check:
                break

        self.control_arm(pose_type="home", speed=30)
        cprint("P=================== 5. Successfully completed the placement task ===================", "green")
        return True

    # ------------------------------------------------------------------
    # Dual-arm trash: left arm transfers to right arm, right arm throws
    # ------------------------------------------------------------------
    def _dual_arm_trash(self):
        """Left→right handover then right arm throws to trash.

        Reuses the validated 4-step handover sequence in `_delegate_to_left_arm`,
        then right arm carries object from home to throw_to_trash_pose and releases.
        """
        cprint("T=================== Dual-arm trash: left → right handover, right throws ===================", "cyan")
        from core.arm import ArmClient
        from core.gripper import GripperClient

        # Phase 1: Validated 4-step handover (ends with both arms at home, object in right gripper)
        if not self._delegate_to_left_arm(container="trash"):
            cprint("T=================== Handover failed ===================", "red")
            return False

        # Phase 2: Right arm throws to trash
        right_cfg = self.config.get_arm_config("right")
        throw_pose = right_cfg.get("throw_to_trash_pose")
        if throw_pose is None:
            cprint("T=================== throw_to_trash_pose not found for right arm ===================", "red")
            return False

        right_arm = self.arm_for("right")
        right_gripper = self.gripper_for("right")

        cprint("T=================== Right arm: home → throw_to_trash_pose (release) → home ===================", "cyan")
        right_arm.move_to_named_pose(throw_pose, speed=15)
        right_gripper.open()
        time.sleep(1)
        right_arm.move_to_named_pose(right_cfg["home"], speed=30)

        cprint("T=================== Dual-arm trash done ===================", "green")
        return True

    def _single_arm_trash(self):
        """Right arm throws trash directly (single arm)."""
        cprint("T=================== Single arm trash: right throws directly ===================", "cyan")

        right_cfg = self.config.get_arm_config("right")
        throw_pose = right_cfg.get("throw_to_trash_pose")
        if throw_pose is None:
            cprint("T=================== throw_to_trash_pose not found for right arm ===================", "red")
            return False

        # Reuse right arm client from grasp phase, or create new one
        right_arm = self.arm_for("right")
        right_gripper = self.gripper_for("right")

        right_arm.move_to_named_pose(throw_pose, speed=15)
        right_gripper.open()
        time.sleep(1)
        right_arm.move_to_named_pose(right_cfg["home"], speed=30)

        cprint("T=================== 5. Successfully completed the single-arm trash task ===================", "green")
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

        self._save_grasp_visualization(
            image, grasp_points, valid_indices, valid_boxes,
            [cls for cls in class_name.split(',')] if class_name else [],
        )

        return final_grasps if ans else []

    def _transform_anygrasp_pose(self, anygrasp_pose, _visualization=True, return_labels=False, side="left"):
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
                # self_pose_matrix (Ry(90°)) was calibrated for the Inspire
                # dexterous hand.  The Robotiq 85 gripper end-effector is
                # oriented 180° differently about its Z axis relative to the
                # dexterous hand palm.  Compensate so J7 wrist roll stays
                # within a natural range.
                if side == "left":
                    rz180 = np.diag([-1.0, -1.0, 1.0, 1.0])
                    self_pose_matrix = self_pose_matrix @ rz180

                transformed_pose_camera = grasp_transformation_matrix @ self_pose_matrix
                transformed_pose_world = self.T_base_to_cam @ transformed_pose_camera
                transformed_pose_world = self._disambiguate_grasp_orientation(transformed_pose_world, side)

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

    def _disambiguate_grasp_orientation(self, transformed_pose_world, side):
        grasp_x_axis = transformed_pose_world[:3, 0]
        if grasp_x_axis[0] < 0:
            flip = np.diag([-1.0, -1.0, 1.0, 1.0])
            transformed_pose_world = transformed_pose_world @ flip
        return transformed_pose_world

    def _execute_grasping_twin_js_2(self, grasping_pose_world_hand, idx=None, side="left", obs_pose=None):
        """Execute grasp trajectory via Twin. Supports left and right grippers."""
        if side == "right":
            arm_client = self._right_arm
            gripper_client = self._right_gripper
            gripper_client.open()
        else:
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
        execution_grasping_pos = execution_grasping_pos @ self.T_hand_effector_to_arm_endlink
        execution_grasping_pos_position = execution_grasping_pos[:3, 3]
        execution_grasping_pos_orientation = R.from_matrix(execution_grasping_pos[:3, :3]).as_quat()

        if side == "right":
            default_traj_js_rad = [v / 180 * np.pi for v in obs_pose.values()]
            struct = self.config.get_arm_config("right").get("twin_struct", "left_arm")
        else:
            default_traj_js_rad = [data / 180 * np.pi for data in self.config.default_traj_js[idx].values()]
            struct = "left_arm"

        cnfg = {
            "target_pose": [
                [preparasion_grasping_pos_position[0], preparasion_grasping_pos_position[1], preparasion_grasping_pos_position[2],
                 preparasion_grasping_pos_orientation[0], preparasion_grasping_pos_orientation[1], preparasion_grasping_pos_orientation[2], preparasion_grasping_pos_orientation[3]],
                [execution_grasping_pos_position[0], execution_grasping_pos_position[1], execution_grasping_pos_position[2],
                 execution_grasping_pos_orientation[0], execution_grasping_pos_orientation[1], execution_grasping_pos_orientation[2], execution_grasping_pos_orientation[3]],
            ],
            "current_js": default_traj_js_rad,
            "struct": struct,
        }
        rsp = self.send_cmd_twin(self.twin_for(side), {"srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg})
        state = rsp["value"]

        if state:
            trajectory_grasping = rsp["info"]["trajectory"]
            trajectory_grasping = np.array(copy.deepcopy(trajectory_grasping)) / np.pi * 180
            if side == "right":
                arm_client.execute_trajectory(trajectory_grasping, speed=20)
                cprint("=============== Reach grasping pose (right) =============")
                gripper_client.close()
                cprint("=============== Close gripper =============")
                time.sleep(0.5)
                arm_client.move_to_named_pose(obs_pose, speed=30)
                cprint("=============== Reach post grasping pose (right) =============")
            else:
                self.control_arm(trajectory=trajectory_grasping, speed=20)
                cprint("=============== Reach grasping pose =============")
                self.control_hand(cmd_type="close")
                cprint("=============== Close hand =============")
                time.sleep(0.5)
                self._left_arm_j2_pretension(speed=15)
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
                center_cam_point = pixel_to_camera_point2(
                    np.array([center_x, center_y]).reshape(-1, 2),
                    mean_depth_m,
                    cam_type="left",
                    intrinsics=self.config.get_camera_intrinsics("left"),
                )
                center_cam_point = center_cam_point.flatten()
                self._placement_rotation = None
                best_grasp = self._select_best_container_grasp(
                    self.rgb, self.depth, (x1, y1, x2, y2), side="left"
                )
                if best_grasp is None:
                    cprint(
                        f"[place] AnyGrasp returned no grasp point inside {class_name}",
                        "yellow",
                    )
                    return []
                self._placement_rotation = best_grasp["rotation"]
                cprint(
                    f"[place] using highest-score AnyGrasp pose for {class_name}: "
                    f"index={best_grasp['index']} score={best_grasp['score']:.4f}",
                    "cyan",
                )
                placing_pos_world = self._transform_pose_to_world(center_cam_point)
                return placing_pos_world
        except Exception:
            pass
        return []

    def _transform_pose_to_world(self, pose_cam_point):
        placing_translation = pose_cam_point.flatten()
        T_cam_point = np.eye(4, 4)
        T_cam_point[:3, :3] = np.eye(3)
        T_cam_point[:3, 3] = placing_translation
        T_world_point = self.T_base_to_cam @ T_cam_point
        T_world_pose = T_world_point.copy()
        if getattr(self, "_placement_rotation", None) is not None:
            T_world_pose[:3, :3] = self._placement_rotation
        return T_world_pose

    def _execute_placement(self, placement_pos_world, initial_js_key="grasp1"):
        placement_pos_world[2, 3] += 0.15
        placement_pos_arm = placement_pos_world @ self.T_hand_effector_to_arm_endlink
        pos = placement_pos_arm[:3, 3]
        orn = R.from_matrix(placement_pos_arm[:3, :3]).as_quat()

        default_js_rad = [v / 180 * np.pi for v in self.config.default_traj_js[initial_js_key].values()]
        cnfg = {
            "target_pose": [[pos[0], pos[1], pos[2], orn[0], orn[1], orn[2], orn[3]]],
            "current_js": default_js_rad,
            "struct": "left_arm",
        }
        rsp = self.send_cmd_twin(self.twin_for("left"), {"srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg})
        if rsp["value"]:
            traj = np.array(copy.deepcopy(rsp["info"]["trajectory"])) / np.pi * 180
            self.control_arm(trajectory=traj, speed=20)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            return True
        return False
