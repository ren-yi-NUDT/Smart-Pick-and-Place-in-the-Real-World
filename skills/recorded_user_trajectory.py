#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared implementation for user-facing recorded trajectory Skills.

Recorded trajectories are data assets, not Skills by themselves.  The public
Skills below provide the registry entry, input validation and the real/sim
backend selection around those assets.
"""

import os
import re
import subprocess
import sys

from termcolor import cprint

from core.drawer_executor import create_drawer_executor
from core.skill_runtime import SkillResult
from skills.base import Skill


class RecordedUserTrajectorySkill(Skill):
    """Replay one validated, single-arm user handover trajectory."""

    ARM_SIDE = "right"
    DEFAULT_TRAJECTORY = ""
    DEFAULT_SPEED = 1.0
    GRIPPER_EVENT_WAIT = 0.0
    ACTION_LABEL = "用户交接轨迹"
    FAILURE_CODE = "USER_TRAJECTORY_FAILED"
    PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    TRAJECTORY_DIR = os.path.join(PROJECT_ROOT, "recorded_trajectories", "right")
    _TRAJECTORY_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")

    def validate_inputs(self, **kwargs):
        trajectory_name = kwargs.get(
            "trajectory_name",
            kwargs.get("trajectory", kwargs.get("name", self.DEFAULT_TRAJECTORY)),
        )
        if not isinstance(trajectory_name, str) or not self._TRAJECTORY_NAME.fullmatch(
                trajectory_name):
            raise ValueError(
                "trajectory_name 只能包含字母、数字、点、下划线和短横线"
            )
        trajectory_path = os.path.join(self.TRAJECTORY_DIR, trajectory_name + ".json")
        if not os.path.isfile(trajectory_path):
            raise FileNotFoundError(
                "%s轨迹不存在: %s" % (self.ACTION_LABEL, trajectory_path)
            )

        speed = float(kwargs.get("speed", self.DEFAULT_SPEED))
        if speed <= 0:
            raise ValueError("speed 必须大于 0")

    def execute(
        self,
        trajectory_name=None,
        speed=None,
        gripper_enabled=True,
        gripper_event_wait=None,
        **kwargs,
    ):
        # Defaults must be resolved on the instance.  Python evaluates a
        # base-class function's default arguments at definition time, so
        # using ``trajectory_name=DEFAULT_TRAJECTORY`` here would ignore a
        # subclass such as ReceiveUserTrajectorySkill or RightGiveToUserSkill.
        if trajectory_name is None:
            trajectory_name = self.DEFAULT_TRAJECTORY
        if speed is None:
            speed = self.DEFAULT_SPEED
        if gripper_event_wait is None:
            gripper_event_wait = self.GRIPPER_EVENT_WAIT
        trajectory_name = kwargs.get(
            "trajectory", kwargs.get("name", trajectory_name)
        )
        speed = float(speed)
        gripper_event_wait = float(gripper_event_wait)
        if gripper_event_wait < 0:
            raise ValueError("gripper_event_wait 必须不小于 0")
        if isinstance(gripper_enabled, str):
            gripper_enabled = gripper_enabled.strip().lower() not in (
                "0", "false", "no", "off"
            )
        else:
            gripper_enabled = bool(gripper_enabled)

        cprint(
            "[%s] %s回放 %s (%.2fx)"
            % (
                self.skill_name,
                "仿真" if self.config.sim_mode else "真机桥接",
                trajectory_name,
                speed,
            ),
            "cyan",
        )
        arm = self.context.arm(self.ARM_SIDE)
        gripper = (
            self.context.gripper(self.ARM_SIDE) if gripper_enabled else None
        )
        executor = create_drawer_executor(
            self.config,
            trajectory_dir=self.TRAJECTORY_DIR,
            arm_client=arm,
            gripper_client=gripper,
        )
        ok = executor.play(
            trajectory_name,
            speed=speed,
            gripper_enabled=gripper_enabled,
            gripper_event_wait=gripper_event_wait,
        )

        if not ok:
            return SkillResult(
                ok=False,
                code=self.FAILURE_CODE,
                message="%s回放失败" % self.ACTION_LABEL,
                recoverable=True,
                data={
                    "arm": self.ARM_SIDE,
                    "trajectory": trajectory_name,
                    "gripper_event_wait": gripper_event_wait,
                },
            )

        return SkillResult(
            ok=True,
            code="SUCCESS",
            message="%s执行完成" % self.ACTION_LABEL,
            data={
                "arm": self.ARM_SIDE,
                "trajectory": trajectory_name,
                "speed": speed,
                "gripper_enabled": gripper_enabled,
                "gripper_event_wait": gripper_event_wait,
                "returned_home": True,
            },
        )

    @property
    def skill_name(self):
        """CLI name used in status messages; subclasses may override it."""
        return self.__class__.__name__.removesuffix("Skill")

    def _run_real_command(self, command):
        """Compatibility helper for external callers of the old CLI path."""
        try:
            completed = subprocess.run(
                [sys.executable] + list(command),
                cwd=self.PROJECT_ROOT,
                check=False,
            )
            return completed.returncode == 0
        except (OSError, subprocess.SubprocessError) as exc:
            cprint("[%s] 回放进程失败: %s" % (self.skill_name, exc), "red")
            return False
