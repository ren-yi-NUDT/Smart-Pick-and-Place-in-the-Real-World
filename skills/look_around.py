import os
from datetime import datetime

from PIL import Image
from termcolor import cprint
from skills.base import Skill, register_skill

from core import scene as scene_map


@register_skill("look_around")
class LookAroundSkill(Skill):
    """Scan workspace from observation positions using GLM-4.5V."""

    def run(self, reset_pose="grasp1", **kwargs):
        """Scan workspace, analyze scene with VLM, and record a scene map.

        Args:
            reset_pose: Pose name to return to after scanning.
                        Set to None to skip reset (for chained calls).
        """
        cprint("=================== Look Around: Scanning workspace ===================", "cyan")

        poses = [k for k in self.config.default_traj_js if "grasp" in k]
        if not poses:
            cprint("No observation positions found", "red")
            return None

        images = {}

        def move_to_pose(name):
            self.control_arm(pose_type=name, speed=30)

        def capture(name):
            rgb, depth = self.get_camera_obs()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            rgb_path = os.path.join(self.save_path, f"look_around_rgb_{timestamp}_{name}.png")
            depth_path = os.path.join(self.save_path, f"look_around_depth_{timestamp}_{name}.png")
            Image.fromarray(rgb).save(rgb_path)
            Image.fromarray(depth).save(depth_path)
            cprint(f"Saved: {rgb_path}", "cyan")
            images[name] = {"rgb": rgb, "depth": depth,
                            "rgb_path": rgb_path, "depth_path": depth_path}
            return rgb, depth

        def transform():
            # 变换必须每个位姿重新取 —— T_base_to_cam 是随臂动的, 沿用上一个位姿会让
            # 所有世界坐标都算错。scan_workspace 的契约要求这里刷新并返回。
            self.save_current_transformation()
            return self.T_base_to_cam

        def detect(rgb):
            return scene_map.detect_scene(self.perception, rgb)

        entries = scene_map.scan_workspace(move_to_pose, capture, transform, detect, poses)
        objects = scene_map.merge(entries)

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

        # 地图**最后**才落盘。save() 盖的是"现在"的时间戳, 而 pick_and_place 拿
        # SCENE_MAP_TTL_S 卡这个时间戳 —— 落早了, 用户在 VLM 上等的那几十秒 (API 抽风时
        # 是几分钟) 就从可用窗口里扣掉了, 命令刚跑完地图就已经过期。
        # 场景在这几十秒里不会变, 所以按"命令返回"计时才是用户真正在用的那个窗口。
        if objects:
            path = scene_map.save(objects)
            cprint(f"\n========== Scene map ({len(objects)} 样东西) -> {path} ==========", "green")
            for o in sorted(objects, key=lambda o: -o["conf"]):
                cprint(f"  {o['color'] or '?'} {o['name']:12s} conf={o['conf']:.2f} "
                       f"world=({o['world'][0]:+.3f},{o['world'][1]:+.3f},{o['world'][2]:+.3f}) "
                       f"从 {'/'.join(o['seen_from'])} 看到", "green")
        else:
            cprint("Scene map: 什么都没扫到", "yellow")
        return analysis
