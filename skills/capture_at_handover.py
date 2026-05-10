import os
import time
from datetime import datetime
from termcolor import cprint
from skills.base import Skill, register_skill


@register_skill("capture_at_handover")
class CaptureAtHandoverSkill(Skill):
    """Move to handover viewing pose, capture image, analyze with VLM."""

    def run(self, reset_pose="grasp1", **kwargs):
        """Move to handover viewing pose, capture image, analyze with VLM.

        Args:
            reset_pose: Pose name to return to after capture.
                        Set to None to skip reset (for chained calls).
        """
        cprint("=================== Capture at Handover ===================", "cyan")

        look_pose = self.config.get_pose("look_over_what_in_user_hand_pose")
        if look_pose is None:
            cprint("错误: robot_config.json 中没有定义 look_over_what_in_user_hand_pose", "red")
            return None

        joint_angles = self.config.pose_to_list(look_pose)
        import numpy as np
        self.control_arm(trajectory=np.array([joint_angles]), speed=15)
        time.sleep(1)

        rgb, depth = self.get_camera_obs()

        # Save images to disk
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        from PIL import Image
        rgb_path = os.path.join(self.save_path, f"handover_rgb_{timestamp}.png")
        depth_path = os.path.join(self.save_path, f"handover_depth_{timestamp}.png")
        Image.fromarray(rgb).save(rgb_path)
        Image.fromarray(depth).save(depth_path)
        cprint(f"Saved: {rgb_path}", "cyan")

        prompt = (
            "画面中有一只手，告诉我手里拿的是什么。请简洁回答，只说物品名称。"
        )

        analysis = self.vlm.analyze(rgb, prompt=prompt)
        if analysis:
            cprint(f"\n========== Object in hand: {analysis} ==========\n", "green")
        else:
            cprint("VLM analysis failed", "red")

        if reset_pose:
            self.control_arm(pose_type=reset_pose, speed=30)
        return analysis
