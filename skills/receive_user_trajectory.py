#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""右臂从用户处取物的预录制轨迹 Skill。

它与 ``right_give_to_user`` 方向相反：本 Skill 闭爪接物，后者开爪递物。
"""

from skills.base import register_skill
from skills.recorded_user_trajectory import RecordedUserTrajectorySkill


@register_skill("receive_user_trajectory")
class ReceiveUserTrajectorySkill(RecordedUserTrajectorySkill):
    """回放右臂取物轨迹，轨迹本身包含闭爪和回 home 段。"""

    ARM_SIDE = "right"
    DEFAULT_TRAJECTORY = "right_receive_user_v3"
    DEFAULT_SPEED = 0.5
    # Hold the end effector at the user's handover position before closing.
    GRIPPER_EVENT_WAIT = 5.0
    ACTION_LABEL = "右臂从用户处取物轨迹"
    FAILURE_CODE = "RECEIVE_TRAJECTORY_FAILED"

    @property
    def skill_name(self):
        return "receive_user_trajectory"
