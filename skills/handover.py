import time
from termcolor import cprint
from skills.base import Skill, register_skill


@register_skill("handover")
class HandoverSkill(Skill):
    """Move to handover pose via waypoints and release object."""

    def run(self, **kwargs):
        pose_1st = self.config.get_pose("get_ready_to_handover_1st")
        pose_2nd = self.config.get_pose("get_ready_to_handover_2nd")
        handover_pose = self.config.get_pose("handover_pose")

        if handover_pose is None:
            cprint("错误: robot_config.json 中缺少 handover_pose 定义", "red")
            return False

        cprint("=============== Moving to handover pose =============", "cyan")
        if pose_1st is not None:
            self.control_arm(pose_type="get_ready_to_handover_1st", speed=15)
        if pose_2nd is not None:
            self.control_arm(pose_type="get_ready_to_handover_2nd", speed=15)
        self.control_arm(pose_type="handover_pose", speed=15)

        cprint("=============== Reached handover pose =============", "green")
        time.sleep(0.5)
        self.control_hand(cmd_type="open")
        time.sleep(2)

        cprint("=============== Retracing path back =============", "cyan")
        if pose_2nd is not None:
            self.control_arm(pose_type="get_ready_to_handover_2nd", speed=15)
        if pose_1st is not None:
            self.control_arm(pose_type="get_ready_to_handover_1st", speed=15)
        self.control_arm(pose_type="home", speed=30)
        cprint("=============== Returned to home =============", "green")
        return True
