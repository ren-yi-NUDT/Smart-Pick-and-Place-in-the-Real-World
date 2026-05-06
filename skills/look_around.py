import os
import time
from datetime import datetime
from termcolor import cprint
from skills.base import Skill, register_skill


@register_skill("look_around")
class LookAroundSkill(Skill):
    """Scan workspace from observation positions using GLM-4.5V."""

    def run(self, **kwargs):
        cprint("=================== Look Around: Scanning workspace ===================", "cyan")

        images = {}
        for key in self.config.default_traj_js:
            if "grasp" not in key:
                continue
            self.control_arm(pose_type=key, speed=30)
            rgb, depth = self.get_camera_obs()

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

        self.control_arm(pose_type="grasp1", speed=30)
        return analysis
