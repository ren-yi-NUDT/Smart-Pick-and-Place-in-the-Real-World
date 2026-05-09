import copy
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core.transforms import pixel_to_camera_point2


@register_skill("fetch_from_user")
class FetchFromUserSkill(Skill):
    """Receive an item from the user and place it."""

    def run(self, **kwargs):
        if kwargs.get("container"):
            json_data = kwargs
        else:
            json_data = self.json_parser.get_command()
        if json_data is None:
            cprint("No valid JSON input received", "red")
            return False

        cprint(f"=================== 1. Get JSON input: {json_data} ===================", "cyan")

        container = json_data.get("container")
        if container is None:
            cprint("JSON input missing required field: container", "red")
            return False

        cprint(f"=================== 2. Target container: {container} ===================", "cyan")

        # ---- Step 2: Move to receive position (handover_pose) ----
        handover_pose = self.config.robot_config.get("handover_pose")
        if handover_pose is None:
            cprint("Error: handover_pose not defined in robot_config.json", "red")
            return False

        cprint("=================== Moving to receive pose (handover) ===================", "cyan")
        joint_angles = [
            handover_pose["J1"], handover_pose["J2"], handover_pose["J3"],
            handover_pose["J4"], handover_pose["J5"], handover_pose["J6"], handover_pose["J7"]
        ]
        trajectory = np.array([joint_angles])
        self.control_arm(trajectory=trajectory, speed=15)
        cprint("=================== Reached receive pose ===================", "green")

        # ---- Step 3: Open hand and wait ----
        self.control_hand(cmd_type="open")
        cprint("=================== 3. Hand opened, ready to receive item ===================", "green")
        time.sleep(1.0)

        # ---- Step 4/5: Close hand and check grasp ----
        has_object = self.check_grasping_object()

        if not has_object:
            cprint("=================== 4. No object detected! Retrying... ===================", "yellow")
            self.control_hand(cmd_type="open")
            time.sleep(3.0)
            has_object = self.check_grasping_object()

            if not has_object:
                cprint("=================== Still no object! Task failed. ===================", "red")
                self.control_arm(pose_type="grasp1", speed=30)
                return False

        cprint("=================== 4. Successfully grabbed object! ===================", "green")
        time.sleep(0.5)

        # ---- Step 6: Placement phase ----
        if container.lower() in ["trash", "垃圾桶", "garbage", "bin"]:
            cprint("=================== Trash mode detected ===================", "cyan")
            throw_pose = self.config.get_pose("throw_to_trash_pose")
            if throw_pose is None:
                cprint("=================== Trash task failed: throw_to_trash_pose not found ===================", "red")
                return False
            joint_angles = [throw_pose[f"J{i}"] for i in range(1, 8)]
            self.control_arm(trajectory=np.array([joint_angles]), speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            self.control_arm(pose_type="grasp1", speed=30)
            cprint("=================== 7. Task completed successfully! ===================", "green")
            return True

        if container.lower() in ["desk", "桌子", "table"]:
            cprint("=================== Desk placement mode detected ===================", "cyan")
            import random
            selected = random.choice(["desk_pose_1", "desk_pose_2", "desk_pose_3"])
            desk_pose = self.config.get_pose(selected)
            if desk_pose is None:
                cprint(f"=================== Desk task failed: {selected} not found ===================", "red")
                return False
            joint_angles = [desk_pose[f"J{i}"] for i in range(1, 8)]
            self.control_arm(trajectory=np.array([joint_angles]), speed=15)
            time.sleep(0.5)
            self.control_hand(cmd_type="open")
            time.sleep(1)
            self.control_arm(pose_type="grasp1", speed=30)
            cprint("=================== 7. Task completed successfully! ===================", "green")
            return True

        # Normal vision-based placement
        check = False
        for key in self.config.default_traj_js.keys():
            if "grasp" not in key:
                continue
            self.control_arm(pose_type=key, speed=30)
            self.rgb, self.depth = self.get_camera_obs()
            self.save_current_transformation()

            cprint(f"=================== 5. Looking for container: {container} ===================", "cyan")
            placing_pos_world = self._get_placing_position(class_name=container, image=self.rgb)

            if not len(placing_pos_world):
                cprint(f"Container not found from pose {key}, trying next...", "yellow")
                continue

            cprint("=================== 6. Container found, executing placement ===================", "cyan")
            check = self._execute_placement(placing_pos_world)
            if check:
                break

        if not check:
            cprint("=================== Placement failed! ===================", "red")
            self.control_arm(pose_type="grasp1", speed=30)
            return False

        self.control_arm(pose_type="grasp1", speed=30)
        cprint("=================== 7. Task completed successfully! ===================", "green")
        return True

    # ------------------------------------------------------------------
    # Placement helpers (from fetch_from_user.py)
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
                x1, y1, x2, y2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)

                depth_sub_image_mm = self.depth[y1:y2, x1:x2]
                valid_depths_mm = depth_sub_image_mm[depth_sub_image_mm > 0]
                if len(valid_depths_mm) > 0:
                    mean_depth_mm = np.median(valid_depths_mm)
                else:
                    return []

                mean_depth_m = mean_depth_mm * 1e-3
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                center_cam_point = pixel_to_camera_point2(
                    np.array([center_x, center_y]).reshape(-1, 2), mean_depth_m
                ).flatten()
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
        return T_world_point.copy()

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
