#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从用户手中接收物品并保持在夹爪中。

这是“用户给机械臂”，不是“机械臂递给用户”；后者使用
``right_give_to_user``（右臂）或 ``handover``（默认左臂）。
"""

from termcolor import cprint

from skills.base import Skill, register_skill


@register_skill("receive_and_hold")
class ReceiveAndHoldSkill(Skill):
    """默认使用右臂接物，验证成功后夹持物品回到 home。"""

    def execute(self, **kwargs):
        side = kwargs.get("side", "right")
        if side not in ("left", "right"):
            cprint("[receive_and_hold] unsupported side: %s" % side, "red")
            return False

        speed_scale = kwargs.get("speed_scale", kwargs.get("speed", 1.0))
        try:
            speed_scale = float(speed_scale)
        except (TypeError, ValueError):
            cprint("[receive_and_hold] invalid speed/speed_scale", "red")
            return False
        if speed_scale <= 0:
            cprint("[receive_and_hold] speed must be positive", "red")
            return False

        ok = self.handover_pipeline.run(
            mode="receive_user",
            side=side,
            wait_seconds=float(kwargs.get("wait_seconds", 10.0)),
            retry_wait_seconds=float(kwargs.get("retry_wait_seconds", 6.0)),
            speed_scale=speed_scale,
        )
        if ok:
            cprint(
                "[receive_and_hold] OK: object held in %s gripper" % side,
                "green",
            )
        else:
            cprint(
                "[receive_and_hold] FAILED: nothing grasped, arm returned home",
                "red",
            )
        return ok
