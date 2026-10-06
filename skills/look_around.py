import os
from datetime import datetime

from PIL import Image
from termcolor import cprint
from skills.base import Skill, register_skill

from core import scene as scene_map


@register_skill("look_around")
class LookAroundSkill(Skill):
    """Scan workspace from observation positions and record a scene map."""

    def run(self, reset_pose="grasp1", **kwargs):
        """Scan workspace and record a scene map.

        Args:
            reset_pose: Pose name to return to after scanning.
                        Set to None to skip reset (for chained calls).

        不再调 VLM: 那段场景文字描述只有人看, 抓取链路一个字段都不用它, 却占掉
        42.8s 里的 6.6s (15%), 而且是唯一会卡住的地方 (API 抽风时重试几分钟)。
        """
        cprint("=================== Look Around: Scanning workspace ===================", "cyan")

        poses = [k for k in self.config.default_traj_js if "grasp" in k]
        if not poses:
            cprint("No observation positions found", "red")
            return None

        # 建图的观察也必须张着夹爪 —— 理由同 pick_and_place: 夹爪在相机视场里,
        # 合拢着拍会挡住物体。而且建图和抓取两边必须是同一个夹爪状态, 否则
        # pick_and_place 拿地图核对现场时会因为画面不一样而误报"东西被挪了"。
        self.control_hand(cmd_type="open")

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

        if reset_pose:
            self.control_arm(pose_type=reset_pose, speed=30)

        # 地图**最后**才落盘。save() 盖的是"现在"的时间戳, 而 pick_and_place 拿
        # SCENE_MAP_TTL_S 卡这个时间戳 —— 落早了, 扫描本身花掉的那 40 秒 (以前还要
        # 加上等 VLM 的几十秒) 就从可用窗口里扣掉了, 命令刚跑完地图就已经过期。
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
        return objects
