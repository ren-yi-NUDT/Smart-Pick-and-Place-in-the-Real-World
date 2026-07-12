import os
import time
from datetime import datetime
from termcolor import cprint
from skills.base import Skill, register_skill


@register_skill("look_around")
class LookAroundSkill(Skill):
    """Scan workspace from observation positions using GLM-4.5V."""

    def run(self, reset_pose=None, **kwargs):
        """Scan workspace and analyze scene with VLM.

        side="left" (default): cycle left arm through grasp1-4 observation poses,
            analyze the first frame with VLM (scene + spatial relations).
        side="right": move right arm to ``drawer_1_placement``, capture the
            drawer interior, ask VLM to list visible items (ignoring foam pads).

        Args:
            reset_pose: Pose name to return to after scanning.
                        None → left arm uses ``grasp1``, right arm uses ``home``.
                        Set to a string to override; pass "" to skip reset.
        """
        data = kwargs if kwargs.get("side") else (self.json_parser.get_command() or {})
        side = data.get("side", "left")

        if side == "right":
            return self._run_right(reset_pose)

        if reset_pose is None:
            reset_pose = "grasp1"
        cprint("=================== Look Around: Scanning workspace (left arm) ===================", "cyan")

        images = {}
        for key in self.config.default_traj_js:
            if "grasp" not in key:
                continue
            self.control_arm(pose_type=key, speed=30)
            rgb, depth = self.get_camera_obs(side="left")

            # Save images to disk
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            from PIL import Image
            rgb_path = os.path.join(self.save_path, f"look_around_rgb_{timestamp}_{key}.png")
            depth_path = os.path.join(self.save_path, f"look_around_depth_{timestamp}_{key}.png")
            Image.fromarray(rgb).save(rgb_path)
            Image.fromarray(depth).save(depth_path)
            cprint(f"Saved: {rgb_path}", "cyan")

            images[key] = {"rgb": rgb, "depth": depth, "rgb_path": rgb_path, "depth_path": depth_path}

        if not images:
            cprint("No observation positions found", "red")
            return None

        # Analyze first image with VLM
        first_key = list(images.keys())[0]
        first_rgb = images[first_key]["rgb"]
        prompt = (
            "请分析图片中的物品及其空间关系，按以下格式回答：\n\n"
            "【物品列表】\n"
            "1. 物品名称 - 位于图片的(左上/右上/左下/右下/中间)位置\n\n"
            "【空间关系】\n"
            "是否有物品被某些容器装着？如果有，请列出，"
            '如"粉红色桃子 在 粉色盘子 里面"\n'
        )

        analysis = self.vlm.analyze(first_rgb, prompt=prompt)
        if analysis:
            cprint(f"\n========== Scene Analysis ==========\n{analysis}\n", "green")
        else:
            cprint("VLM analysis failed", "red")

        if reset_pose:
            self.control_arm(pose_type=reset_pose, speed=30)
        return analysis

    def _run_right(self, reset_pose=None):
        """Right-arm drawer inspection: capture drawer interior, list contents via VLM."""
        from core.arm import ArmClient

        cprint("=================== Look Around: Drawer inspection (right arm) ===================", "cyan")

        right_cfg = self.config.get_arm_config("right")
        obs_pose = right_cfg.get("drawer_1_placement")
        if obs_pose is None:
            cprint("错误: robot_config.json 中右臂没有定义 drawer_1_placement", "red")
            return None

        right_arm = ArmClient("127.0.0.1", 8011)
        right_arm.connect()
        right_arm.move_to_named_pose(obs_pose, speed=15)
        time.sleep(1)

        rgb, depth = self.get_camera_obs(side="right")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        from PIL import Image
        rgb_path = os.path.join(self.save_path, f"look_around_drawer_rgb_{timestamp}.png")
        depth_path = os.path.join(self.save_path, f"look_around_drawer_depth_{timestamp}.png")
        Image.fromarray(rgb).save(rgb_path)
        Image.fromarray(depth).save(depth_path)
        cprint(f"Saved: {rgb_path}", "cyan")

        prompt = (
            "这是一张抽屉内部的照片，请你描述在抽屉里都看到了什么，"
            "只输出词语即可，比如\"桃子、可乐瓶\"，忽视抽屉里的泡沫板"
        )

        analysis = self.vlm.analyze(rgb, prompt=prompt)
        if analysis:
            cprint(f"\n========== Drawer Contents ==========\n{analysis}\n", "green")
        else:
            cprint("VLM analysis failed", "red")

        target = reset_pose if reset_pose else "home"
        target_pose = right_cfg.get(target)
        if target_pose is not None:
            right_arm.move_to_named_pose(target_pose, speed=30)
        return analysis
