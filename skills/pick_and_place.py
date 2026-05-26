import copy
import random
import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core.transforms import graspcam2pixel, self_rotation_np, rpy_to_vector, pixel_to_camera_point2


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
            return self._place_desk()

        if side == "right":
            if self._is_right_fixed_placement(container):
                return self._place_predefined(container)
            return self._delegate_to_left_arm(container)

        return self._place_visual(container)

    # ------------------------------------------------------------------
    # Placement routing helpers
    # ------------------------------------------------------------------
    def _is_right_fixed_placement(self, container):
        """Check if container is a right-arm fixed placement target (no handover needed)."""
        return container.lower() in ["drawer", "cabinet"]

    def _place_predefined(self, pose_name):
        """Right arm moves to a predefined pose and releases the object."""
        cprint(f"P=================== Right arm fixed placement: {pose_name} ===================", "cyan")
        right_cfg = self.config.get_arm_config("right")
        right_arm = getattr(self, "_right_arm", None)
        right_gripper = getattr(self, "_right_gripper", None)

        pose = right_cfg.get(pose_name)
        if pose is None:
            cprint(f"P=================== Pose '{pose_name}' not found in right arm config ===================", "red")
            return False

        right_arm.move_to_named_pose(pose, speed=15)
        if right_gripper:
            right_gripper.open()
        time.sleep(1)
        right_arm.move_to_named_pose(right_cfg["home"], speed=30)
        cprint(f"P=================== Fixed placement to {pose_name} done ===================", "green")
        return True

    def _delegate_to_left_arm(self, container):
        """Right arm hands over to left arm, then left arm places (visual or fixed)."""
        cprint(f"D=================== Delegating to left arm for placement: {container} ===================", "cyan")
        from core.arm import ArmClient
        from core.gripper import GripperClient

        handover = self.config.get_dual_arm_pose("right_to_left_handover")
        if handover is None:
            cprint("D=================== right_to_left_handover pose not found ===================", "red")
            return False

        right_arm = getattr(self, "_right_arm", None)
        if right_arm is None:
            right_arm = ArmClient("127.0.0.1", 8011)
            right_arm.connect()
        right_gripper = getattr(self, "_right_gripper", None)
        if right_gripper is None:
            right_gripper = GripperClient("127.0.0.1", 8001)
            right_gripper.connect()

        # Both arms to handover
        cprint("D=================== Moving both arms to right→left handover ===================", "cyan")
        t1 = threading.Thread(target=self.arm.move_to_named_pose, args=(handover["left_pose"],), kwargs={"speed": 15})
        t2 = threading.Thread(target=right_arm.move_to_named_pose, args=(handover["right_pose"],), kwargs={"speed": 15})
        t1.start(); t2.start(); t1.join(); t2.join()

        # Transfer: right opens, left closes
        right_gripper.open()
        self.control_hand(cmd_type="close")
        time.sleep(2)

        # Right home + left placement in parallel
        cprint("D=================== Right→home, Left→placement ===================", "cyan")

        def _left_placement():
            if container.lower() in ["desk", "桌子", "table"]:
                self._place_desk()
            else:
                self._place_visual(container)

        right_cfg = self.config.get_arm_config("right")
        t1 = threading.Thread(target=_left_placement)
        t2 = threading.Thread(target=right_arm.move_to_named_pose, args=(right_cfg["home"],), kwargs={"speed": 30})
        t1.start(); t2.start(); t1.join(); t2.join()

        cprint("D=================== Delegated placement done ===================", "green")
        return True

    # ------------------------------------------------------------------
    def _visual_grasp_phase(self, obj, side="left", location="desk_front"):
        """Visual grasp. Left arm: cycle observation poses. Right arm: single location."""
        if side == "right":
            return self._visual_grasp_right(obj, location)
        # Left arm: cycle through observation poses
        self.control_arm(pose_type="grasp1", speed=30)
        self.control_hand(cmd_type="close")

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
        return check

    def _visual_grasp_right(self, obj, location="desk_front"):
        """Right arm visual grasp: single observation pose, no cycling."""
        from core.arm import ArmClient
        from core.gripper import GripperClient

        right_cfg = self.config.get_arm_config("right")
        obs_pose = right_cfg["default_traj_js"].get(location)
        if obs_pose is None:
            cprint(f"G=================== Right arm pose '{location}' not found ===================", "red")
            return False

        self._right_arm = ArmClient("127.0.0.1", 8011)
        self._right_arm.connect()
        self._right_gripper = GripperClient("127.0.0.1", 8001)
        self._right_gripper.connect()

        # Move to observation pose
        self._right_arm.move_to_named_pose(obs_pose, speed=20)
        self._right_gripper.open()

        # Capture from right camera
        cam = self.get_camera("right")
        rgb, depth = cam.get_rgbd()
        cprint(f"G=================== [right:{location}] Captured RGB-D ===================", "cyan")

        # Save right arm TF transforms
        self._save_right_arm_transforms()

        # Detect grasps
        anygrasp_pose = self.perception.detect_grasps(rgb, depth)
        if not anygrasp_pose:
            cprint("G=================== No grasp candidates ===================", "red")
            return False

        filtering = self._filtering_pose(anygrasp_pose, class_name=obj, image=rgb)
        if not filtering:
            cprint(f"G=================== No filtered grasps for '{obj}' ===================", "red")
            return False

        grasping_poses = self._transform_anygrasp_pose(filtering, _visualization=False, side="right")
        if not len(grasping_poses):
            cprint("G=================== No transformed poses ===================", "red")
            return False

        # Try each grasp pose
        check = False
        for i, gp in enumerate(grasping_poses):
            cprint(f"=================== Checking pose: {i+1} / {len(grasping_poses)} ===================", "yellow")
            check = self._execute_grasping_twin_js_2(gp, side="right", obs_pose=obs_pose)
            if check:
                break

        if not check:
            cprint("G=================== Right arm grasping task failed ===================", "red")
        return check

    def _save_right_arm_transforms(self):
        """Cache TF transforms for right arm."""
        right_cfg = self.config.get_arm_config("right")
        self.T_base_to_cam, _, _ = (
            self.transforms.get_transform_from_frame_to_frame(
                right_cfg["base_link_name"], "R_cam_link_grasp")
        )
        self.T_hand_effector_to_arm_endlink, _, _ = (
            self.transforms.get_transform_from_frame_to_frame(
                right_cfg["hand_effector_name"], right_cfg["arm_end_link_name"])
        )

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
            from core.arm import ArmClient
            from core.gripper import GripperClient

            handover = self.config.get_dual_arm_pose("right_to_left_handover")
            if handover is None:
                cprint("H=================== right_to_left_handover pose not found ===================", "red")
                return False

            right_arm = getattr(self, "_right_arm", None)
            if right_arm is None:
                right_arm = ArmClient("127.0.0.1", 8011)
                right_arm.connect()
            right_gripper = getattr(self, "_right_gripper", None)
            if right_gripper is None:
                right_gripper = GripperClient("127.0.0.1", 8001)
                right_gripper.connect()

            # Both arms to handover positions in parallel
            cprint("H=================== Moving both arms to right→left handover ===================", "cyan")
            t1 = threading.Thread(target=self.arm.move_to_named_pose, args=(handover["left_pose"],), kwargs={"speed": 15})
            t2 = threading.Thread(target=right_arm.move_to_named_pose, args=(handover["right_pose"],), kwargs={"speed": 15})
            t1.start(); t2.start(); t1.join(); t2.join()

            # Transfer: right opens, left closes
            right_gripper.open()
            self.control_hand(cmd_type="close")
            time.sleep(2)

            # Parallel: right→home + left→handover to person
            cprint("H=================== Right→home, Left→handover to person ===================", "cyan")

            def _left_handover_to_person():
                pose_1st = self.config.get_pose("get_ready_to_handover_1st")
                pose_2nd = self.config.get_pose("get_ready_to_handover_2nd")
                if pose_1st is not None:
                    self.arm.move_to_named_pose(pose_1st, speed=15)
                if pose_2nd is not None:
                    self.arm.move_to_named_pose(pose_2nd, speed=15)
                self.arm.move_to_named_pose(self.config.get_pose("handover_pose"), speed=15)
                self.control_hand(cmd_type="open")
                time.sleep(2)
                if pose_2nd is not None:
                    self.arm.move_to_named_pose(pose_2nd, speed=15)
                if pose_1st is not None:
                    self.arm.move_to_named_pose(pose_1st, speed=15)
                self.arm.move_to_named_pose(self.config.get_arm_config("left")["home"], speed=30)

            right_cfg = self.config.get_arm_config("right")
            t1 = threading.Thread(target=_left_handover_to_person)
            t2 = threading.Thread(target=right_arm.move_to_named_pose, args=(right_cfg["home"],), kwargs={"speed": 30})
            t1.start(); t2.start(); t1.join(); t2.join()

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
            check = self._execute_placement(placing_pos_world)
            if check:
                break

        self.control_arm(pose_type="home", speed=30)
        cprint("P=================== 5. Successfully completed the placement task ===================", "green")
        return True

    # ------------------------------------------------------------------
    # Dual-arm trash: left arm transfers to right arm, right arm throws
    # ------------------------------------------------------------------
    def _dual_arm_trash(self):
        cprint("T=================== Dual-arm trash: left → right handover, right throws ===================", "cyan")
        from core.arm import ArmClient
        from core.gripper import GripperClient
        from core.transition import plan_transition_sequence

        handover = self.config.get_dual_arm_pose("left_to_right_handover")
        if handover is None:
            cprint("T=================== left_to_right_handover pose not found ===================", "red")
            return False
        right_cfg = self.config.get_arm_config("right")
        right_adj = right_cfg.get("transition_adjacency", {})

        right_arm = ArmClient("127.0.0.1", 8011)
        right_arm.connect()
        right_gripper = GripperClient("127.0.0.1", 8001)
        right_gripper.connect()

        # Both arms to handover poses
        cprint("T=================== Moving both arms to handover ===================", "cyan")
        t1 = threading.Thread(target=self.arm.move_to_named_pose, args=(handover["left_pose"],), kwargs={"speed": 15})
        t2 = threading.Thread(target=right_arm.move_to_named_pose, args=(handover["right_pose"],), kwargs={"speed": 15})
        t1.start(); t2.start(); t1.join(); t2.join()

        # Transfer: left opens, right closes
        self.control_hand(cmd_type="open")
        right_gripper.close()
        time.sleep(2)

        # Plan right arm sequence with transition adjacency
        right_seq = plan_transition_sequence(
            ["left_to_right_handover", "throw_to_trash_pose", "home"],
            right_adj
        )
        cprint(f"T=================== Right arm planned: {right_seq} ===================", "cyan")

        # Parallel: left home + right executes trash sequence
        def right_execute():
            for pose_name in right_seq:
                if pose_name == "left_to_right_handover":
                    pose = handover["right_pose"]
                else:
                    pose = right_cfg.get(pose_name)
                if pose:
                    right_arm.move_to_named_pose(pose, speed=15)
                    if pose_name == "throw_to_trash_pose":
                        right_gripper.open()

        t1 = threading.Thread(target=self.arm.move_to_named_pose, args=(self.config.get_arm_config("left")["home"],), kwargs={"speed": 30})
        t2 = threading.Thread(target=right_execute)
        t1.start(); t2.start(); t1.join(); t2.join()

        cprint("T=================== 5. Successfully completed the dual-arm trash task ===================", "green")
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
        right_arm = getattr(self, "_right_arm", None)
        if right_arm is None:
            from core.arm import ArmClient
            right_arm = ArmClient("127.0.0.1", 8011)
            right_arm.connect()
        right_gripper = getattr(self, "_right_gripper", None)

        right_arm.move_to_named_pose(throw_pose, speed=15)
        if right_gripper:
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

                transformed_pose_camera = grasp_transformation_matrix @ self_pose_matrix
                transformed_pose_world = self.T_base_to_cam @ transformed_pose_camera
                if side == "left":
                    transformed_pose_world = self._transform_x_axis(transformed_pose_world)
                elif side == "right":
                    transformed_pose_world = self._transform_right_grasp(transformed_pose_world)

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

    def _transform_right_grasp(self, transformed_pose_world):
        r, p, y = R.from_matrix(transformed_pose_world[:3, :3]).as_euler('xyz', degrees=False)
        y_axis_rotated = rpy_to_vector(r, p, y, axis=[0, 1, 0])
        z_axis_world = np.array([0, 0, 1])
        cos_theta = np.dot(y_axis_rotated, z_axis_world) / (
            np.linalg.norm(y_axis_rotated) * np.linalg.norm(z_axis_world)
        )
        if cos_theta > 0:
            transformed_pose_world = transformed_pose_world @ np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        return transformed_pose_world

    def _execute_grasping_twin_js_2(self, grasping_pose_world_hand, idx=None, side="left", obs_pose=None):
        """Execute grasp trajectory via Twin. Supports left (dexterous hand) and right (gripper)."""
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
        execution_grasping_pos[2, 3] = max(execution_grasping_pos[2, 3], 0.042)
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
        rsp = self.send_cmd_twin(self.twin, {"srv": "twin_inference", "type": "trajectory_generation2", "cnfg": cnfg})
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
