import time
import numpy as np
from termcolor import cprint
from skills.base import Skill, register_skill


@register_skill("handover")
class HandoverSkill(Skill):
    """Move to handover pose via smooth interpolated trajectory and release object."""

    def run(self, **kwargs):
        pose_1st = self.config.get_pose("get_ready_to_handover_1st")
        pose_2nd = self.config.get_pose("get_ready_to_handover_2nd")
        handover_pose = self.config.get_pose("handover_pose")

        if pose_1st is None or pose_2nd is None or handover_pose is None:
            cprint("错误: robot_config.json 中缺少 handover 相关位姿定义", "red")
            return False

        cprint("=============== Moving to handover pose (smooth trajectory) =============", "cyan")

        def extract_joints(pose_dict):
            return [pose_dict[f"J{i}"] for i in range(1, 8)]

        waypoint_joints = [
            extract_joints(pose_1st),
            extract_joints(pose_2nd),
            extract_joints(handover_pose)
        ]

        trajectory = self._interpolate_joint_trajectory(waypoint_joints, steps_per_segment=20)
        self.control_arm(trajectory=trajectory, speed=15)

        cprint("=============== Reached handover pose =============", "green")
        time.sleep(0.5)
        self.control_hand(cmd_type="open")
        time.sleep(0.3)
        hand_opened = self.hand.is_fully_open()
        cprint(f"=============== Hand fully open: {hand_opened} =============",
               "green" if hand_opened else "yellow")
        time.sleep(0.7)
        self.control_arm(pose_type="grasp1", speed=30)
        return hand_opened

    def _interpolate_joint_trajectory(self, waypoint_joints, steps_per_segment=20):
        trajectory = []
        for i in range(len(waypoint_joints) - 1):
            start_joints = np.array(waypoint_joints[i])
            end_joints = np.array(waypoint_joints[i + 1])
            for step in range(steps_per_segment):
                t = step / steps_per_segment
                trajectory.append(start_joints + t * (end_joints - start_joints))
        trajectory.append(np.array(waypoint_joints[-1]))
        return np.array(trajectory)
