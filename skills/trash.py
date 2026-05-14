import time
import numpy as np
from termcolor import cprint
from skills.base import Skill, register_skill


@register_skill("trash")
class TrashSkill(Skill):
    """Move to trash pose and release object."""

    def run(self, **kwargs):
        throw_pose = self.config.get_pose("throw_to_trash_pose")
        if throw_pose is None:
            cprint("错误: robot_config.json 中没有定义 throw_to_trash_pose", "red")
            return False

        cprint("=============== Moving to trash pose =============", "cyan")
        joint_angles = [throw_pose[f"J{i}"] for i in range(1, 8)]
        self.control_arm(trajectory=np.array([joint_angles]), speed=15)
        time.sleep(0.5)
        self.control_hand(cmd_type="open")
        time.sleep(0.3)
        hand_opened = self.hand.is_fully_open()
        cprint(f"=============== Hand fully open: {hand_opened} =============",
               "green" if hand_opened else "yellow")
        time.sleep(0.7)
        self.control_arm(pose_type="grasp1", speed=30)
        return hand_opened
