import copy
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from skills.base import Skill, register_skill
from core.transforms import pixel_to_camera_point2


@register_skill("place")
class PlaceSkill(Skill):
    """Vision-based placement: detect container, compute 3D position, execute."""

    def run(self, **kwargs):
        container = kwargs.get("container")
        if container is None:
            cprint("place skill: missing 'container' parameter", "red")
            return False

        for key, value in self.config.default_traj_js.items():
            if "grasp" in key:
                self.control_arm(pose_type=key, speed=30)
                self.rgb, self.depth = self.get_camera_obs()
                self.save_current_transformation()
                placing_pos_world = self._get_placing_position(container, self.rgb)
                if not len(placing_pos_world):
                    continue
                check = self._execute_placement(placing_pos_world)
                if check:
                    break

        self.control_arm(pose_type="grasp1", speed=30)
        return True

    def _get_placing_position(self, class_name, image):
        detections = self.perception.detect_objects(image, [class_name], conf=0.25)
        try:
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
            center_cam = pixel_to_camera_point2(
                np.array([center_x, center_y]).reshape(-1, 2), mean_depth_m
            ).flatten()
            return self._transform_pose_to_world(center_cam)
        except Exception:
            return []

    def _transform_pose_to_world(self, pose_cam_point):
        T_cam_point = np.eye(4)
        T_cam_point[:3, :3] = R.from_quat([-0.210, 0.016, -0.056, 0.976]).as_matrix()
        T_cam_point[:3, 3] = pose_cam_point.flatten()
        return self.T_base_to_cam @ T_cam_point

    def _execute_placement(self, placement_pos_world):
        placement_pos_world[2, 3] += 0.15
        placement_pos_arm = placement_pos_world @ self.T_hand_effector_to_arm_endlink
        pos = placement_pos_arm[:3, 3]
        orn = R.from_matrix(placement_pos_arm[:3, :3]).as_quat()
        cnfg = {
            "target_pose": [[pos[0], pos[1], pos[2], orn[0], orn[1], orn[2], orn[3]]],
            "current_js": [v / 180 * np.pi for v in self.config.default_traj_js["grasp1"].values()],
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
