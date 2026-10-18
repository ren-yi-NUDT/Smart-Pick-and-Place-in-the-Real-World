"""独立视觉抓取入口。复刻原项目 GraspSkill: 重试循环包住管线。

管线 (core/grasp_pipeline.py) 自带失败恢复 —— 候选/观察位失败后自己回观察位
张爪 —— 这里只负责失败后带着新鲜感知再试一轮。
"""
from termcolor import cprint

from skills.base import Skill, register_skill


@register_skill("grasp")
class GraspSkill(Skill):
    """RGB-D → AnyGrasp → Twin → 夹爪 的视觉抓取。"""

    def run(self, **kwargs):
        data = kwargs if kwargs.get("object") else self.json_parser.get_command()
        if not data or not data.get("object"):
            cprint("[grasp] 缺少 object", "red")
            return False
        max_attempts = max(1, int(data.get("max_attempts", 2)))
        hold_after_grasp = bool(data.get("hold_after_grasp", False))
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                cprint(
                    f"[grasp] 视觉抓取失败，开始第 {attempt} 次尝试 / {max_attempts}",
                    "yellow",
                )
            if self.grasp_pipeline.visual_grasp(
                data["object"],
                hold_after_grasp=hold_after_grasp,
            ):
                return True
        cprint(f"[grasp] 视觉抓取在 {max_attempts} 次尝试后仍失败", "red")
        return False
