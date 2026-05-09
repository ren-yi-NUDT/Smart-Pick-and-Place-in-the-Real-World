import time
import numpy as np
from termcolor import cprint
from skills.base import Skill, register_skill


@register_skill("trash")
class TrashSkill(Skill):
    """Move to trash pose and release object."""

    def run(self, **kwargs):
        if self.config.get_pose("throw_to_trash_pose") is None:
            cprint("错误: robot_config.json 中没有定义 throw_to_trash_pose", "red")
            return False

        cprint("=============== Moving to trash pose =============", "cyan")
        self.control_arm(pose_type="throw_to_trash_pose", speed=15)
        time.sleep(0.5)
        self.control_hand(cmd_type="open")
        time.sleep(1)
        self.control_arm(pose_type="grasp1", speed=30)
        return True
