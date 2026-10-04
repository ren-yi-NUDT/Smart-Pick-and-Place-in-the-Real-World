import os
from datetime import datetime

import numpy as np
from termcolor import cprint
from skills.base import Skill, register_skill

from core import scene as scene_map
from core.config import YOLO_CONF_SCAN
from core.perception import dominant_color_name
from core.transforms import pixel_to_camera_point2


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

        images = {}
        entries = []
        for key in self.config.default_traj_js:
            if "grasp" not in key:
                continue
            self.control_arm(pose_type=key, speed=30)
            # 变换必须每个位姿重新取 —— T_base_to_cam 是随臂动的, 沿用上一个位姿
            # 会让所有世界坐标都算错。get_camera_obs 不会自动刷新, 得显式调。
            self.save_current_transformation()
            rgb, depth = self.get_camera_obs()

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            from PIL import Image
            rgb_path = os.path.join(self.save_path, f"look_around_rgb_{timestamp}_{key}.png")
            depth_path = os.path.join(self.save_path, f"look_around_depth_{timestamp}_{key}.png")
            Image.fromarray(rgb).save(rgb_path)
            Image.fromarray(depth).save(depth_path)
            cprint(f"Saved: {rgb_path}", "cyan")

            images[key] = {"rgb": rgb, "depth": depth, "rgb_path": rgb_path, "depth_path": depth_path}
            entries += self._scan_pose(rgb, depth, key)

        if not images:
            cprint("No observation positions found", "red")
            return None

        # 记住东西在哪 —— 不然下次用户说"放粉盘子里"还得靠臂正好看着它
        objects = scene_map.merge(entries)
        if objects:
            path = scene_map.save(objects)
            cprint(f"\n========== Scene map ({len(objects)} 样东西) -> {path} ==========", "green")
            for o in sorted(objects, key=lambda o: -o["conf"]):
                cprint(f"  {o['color'] or '?'} {o['name']:12s} conf={o['conf']:.2f} "
                       f"world=({o['world'][0]:+.3f},{o['world'][1]:+.3f},{o['world'][2]:+.3f}) "
                       f"从 {'/'.join(o['seen_from'])} 看到", "green")
        else:
            cprint("Scene map: 什么都没扫到", "yellow")

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

    def _scan_pose(self, rgb, depth, pose_key):
        """在这个位姿上找东西, 返回带世界坐标的记录。"""
        detections = self.perception.detect_objects(rgb, scene_map.SCAN_VOCAB, conf=YOLO_CONF_SCAN)
        out = []
        for d in detections:
            world = self._world_point(rgb, depth, d[:4])
            if world is None:
                continue
            out.append({
                "name": scene_map.SCAN_VOCAB[int(d[5])],
                "color": dominant_color_name(rgb, d),
                "world": world,
                "conf": float(d[4]),
                "seen_from": [pose_key],
            })
        return out

    def _world_point(self, rgb, depth, box):
        """框中心的深度 -> 世界坐标。深度全空时返回 None。"""
        H, W = depth.shape
        x1, y1, x2, y2 = [int(v) for v in box]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(W, x2), min(H, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        sub = depth[y1:y2, x1:x2]
        valid = sub[sub > 0]
        if not len(valid):
            return None
        point = pixel_to_camera_point2(
            np.array([[(x1 + x2) / 2, (y1 + y2) / 2]]),
            float(np.median(valid)) * 1e-3,
        ).flatten()
        T = np.eye(4)
        T[:3, 3] = point
        return (self.T_base_to_cam @ T)[:3, 3].tolist()
