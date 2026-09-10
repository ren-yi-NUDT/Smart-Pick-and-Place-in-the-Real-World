#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""擦桌子技能（完整流程：抓海绵 → 回 home → 擦桌 → 放回海绵）。

三个有明确顺序的阶段：

1. 固定位姿抓取海绵：张爪 → home → 观察位 → 抓取预备位 → 抓取位 →
   最低力（force=1）闭合，随后沿抓取逆路径经抓取预备位、观察位回到
   home；
2. 回放右臂的 ``wipe_table_demo`` 录制轨迹（夹爪不受控，保持低力夹持）；
3. 抓取逆操作放回海绵：预备位 → 抓取位 → 张爪 → 预备位 → home。

抓取使用固定位姿而非视觉抓取——擦桌效果依赖海绵在夹爪中的位姿相对
不变；闭爪后仅通过夹爪状态确认是否抓取成功。

轨迹回放：真机经 ``tools/record_sequence.traj_play`` 按**录制时间戳**
回放（speed 为拖动速度的倍率，并自动先移到轨迹起点）；仿真模式走
``core.drawer_executor`` 的 PyBullet 执行器。
全程复用本 Skill 实例的臂/夹爪连接，不创建新 Skill 实例。

CLI 示例：
    python3 run_skill.py wipe_table                     # 完整流程
    echo '{"grasp_sponge": false}' | python3 run_skill.py wipe_table   # 只擦（需已夹海绵）
    echo '{"speed": 0.5, "home_speed": 10}' | python3 run_skill.py wipe_table
    SIM_MODE=1 python3 run_skill.py wipe_table
"""

import os
import re

from termcolor import cprint

from core.drawer_executor import create_drawer_executor
from core.skill_runtime import SkillResult
from skills.base import Skill, register_skill
from skills.pose_execute import _load_poses


@register_skill("wipe_table")
class WipeTableSkill(Skill):
    """抓海绵 → 逆路径回 home → 擦桌轨迹 → 放回海绵。"""

    ARM_SIDE = "right"
    DEFAULT_TRAJECTORY = "wipe_table_demo"
    DEFAULT_SPEED = 0.5
    DEFAULT_HOME_SPEED = 10
    _TRAJECTORY_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")
    PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # 海绵固定抓取流程参数（实测验证值，勿随意改动）
    SPONGE_GRASP_FORCE = 1   # 最低非零夹持力，防止海绵形变
    GRIPPER_SPEED = 10
    SPEED_HOME = 15
    SPEED_TRAVEL = 10
    SPEED_GRASP = 8

    def validate_inputs(self, **kwargs):
        """在建立硬件连接前校验运动参数。"""
        trajectory_name = kwargs.get(
            "trajectory_name",
            kwargs.get("trajectory", kwargs.get("name", self.DEFAULT_TRAJECTORY)),
        )
        if not isinstance(trajectory_name, str) or not self._TRAJECTORY_NAME.fullmatch(
            trajectory_name
        ):
            raise ValueError(
                "trajectory_name 只能包含字母、数字、点、下划线和短横线"
            )

        speed = float(kwargs.get("speed", self.DEFAULT_SPEED))
        if speed <= 0:
            raise ValueError("speed 必须大于 0")

        home_speed = int(kwargs.get("home_speed", self.DEFAULT_HOME_SPEED))
        if not 1 <= home_speed <= 100:
            raise ValueError("home_speed 必须在 1 到 100 之间")

    # ------------------------------------------------------------------
    # 轨迹回放
    # ------------------------------------------------------------------

    @staticmethod
    def _play_real(name, speed, gripper_enabled, skip_ranges):
        """真机回放：SDK 按录制时间戳流式执行，speed 为录制速度的倍率。

        drawer_executor 的 speed 是控制器速度百分比（0.5x → 10%），
        与录制节奏无关，回放不出"拖动速度的 0.5 倍"；这里复用
        record_sequence.traj-play 的时间戳回放（含自动移到轨迹起点）。
        """
        from tools.record_sequence import traj_play
        return traj_play(
            name,
            speed=speed,
            gripper_enabled=gripper_enabled,
            skip_ranges=skip_ranges,
        )

    def _play_trajectory(self, trajectory_name, speed, gripper_enabled, skip_ranges):
        cprint(
            f"[wipe_table] 阶段2: "
            f"{'仿真' if self.config.sim_mode else '真机SDK时间戳'}回放 "
            f"{trajectory_name} ({speed}x)",
            "cyan",
        )
        try:
            if self.config.sim_mode:
                executor = create_drawer_executor(
                    self.config,
                    arm_client=self.context.arm(self.ARM_SIDE),
                    gripper_client=self.context.gripper(self.ARM_SIDE),
                )
                return executor.play(
                    trajectory_name,
                    speed=speed,
                    skip_ranges=skip_ranges,
                    gripper_enabled=gripper_enabled,
                )
            return self._play_real(
                trajectory_name, speed, gripper_enabled, skip_ranges
            )
        except (TypeError, ValueError) as exc:
            cprint(f"[wipe_table] 轨迹参数无效: {exc}", "red")
            return False

    # ------------------------------------------------------------------
    # 海绵抓取 / 放回
    # ------------------------------------------------------------------

    def _play_teach_pose(self, name, speed):
        """经本实例的臂连接执行 recorded_poses 位姿（不新建连接）。"""
        pose = _load_poses(self.ARM_SIDE).get(name)
        if not pose:
            cprint(f"[wipe_table] recorded_poses 中没有 {name}", "red")
            return False
        joints = pose["joint_angles_deg"]
        pose_dict = {f"J{i + 1}": float(v) for i, v in enumerate(joints)}
        ok = self.context.arm(self.ARM_SIDE).move_to_named_pose(
            pose_dict, speed=speed
        )
        if not ok:
            cprint(f"[wipe_table] [{self.ARM_SIDE}] 执行位姿 {name} 失败", "red")
        return ok

    def _grasp_sponge(self):
        """阶段1：固定抓取海绵。返回 None 表示成功，否则为错误码。"""
        cprint("[wipe_table] 阶段1: 固定位姿抓取海绵", "cyan")
        side = self.ARM_SIDE
        gripper = self.context.gripper(side)
        gripper.open(speed=self.GRIPPER_SPEED)

        if not self._play_teach_pose("home", self.SPEED_HOME):
            return "HOME_MOVE_FAILED"
        if not self._play_teach_pose("sponge_observation_pose", self.SPEED_TRAVEL):
            return "OBSERVE_POSE_FAILED"

        if not self._play_teach_pose("sponge_grasp_pre_pose", self.SPEED_TRAVEL):
            return "GRASP_PRE_FAILED"
        if not self._play_teach_pose("sponge_grasp_pose", self.SPEED_GRASP):
            return "GRASP_POSE_FAILED"

        gripper.close(force=self.SPONGE_GRASP_FORCE, speed=self.GRIPPER_SPEED)
        if not gripper.is_grasping(force=self.SPONGE_GRASP_FORCE):
            cprint("[wipe_table] STOP: 夹爪未检测到海绵", "red")
            return "SPONGE_GRASP_FAILED"
        return None

    def _return_home_after_grasp(self):
        """抓取成功后沿固定抓取路径的逆序安全退回 home。"""
        cprint("[wipe_table] 抓取完成，沿抓取逆路径回 home", "cyan")
        return (
            self._play_teach_pose("sponge_grasp_pre_pose", self.SPEED_TRAVEL)
            and self._play_teach_pose("sponge_observation_pose", self.SPEED_TRAVEL)
            and self._play_teach_pose("home", self.SPEED_HOME)
        )

    def _put_back_sponge(self):
        """阶段3：抓取逆操作放回海绵。"""
        cprint("[wipe_table] 阶段3: 放回海绵（抓取逆操作）", "cyan")
        side = self.ARM_SIDE
        if not self._play_teach_pose("sponge_grasp_pre_pose", self.SPEED_TRAVEL):
            return False
        if not self._play_teach_pose("sponge_grasp_pose", self.SPEED_GRASP):
            return False
        self.context.gripper(side).open(speed=self.GRIPPER_SPEED)
        return self._play_teach_pose("sponge_grasp_pre_pose", self.SPEED_TRAVEL)

    # ------------------------------------------------------------------
    # 编排
    # ------------------------------------------------------------------

    def execute(
        self,
        trajectory_name=DEFAULT_TRAJECTORY,
        speed=DEFAULT_SPEED,
        return_home=True,
        home_speed=DEFAULT_HOME_SPEED,
        skip_ranges=None,
        gripper_enabled=False,
        grasp_sponge=True,
        put_back=None,
        **kwargs,
    ):
        """执行完整擦桌子流程，按顺序编排三个阶段。

        ``trajectory`` 和 ``name`` 是 ``trajectory_name`` 的兼容别名。

        ``grasp_sponge``: 阶段1 固定抓取海绵并沿逆路径回 home（默认 True）。
        ``put_back``: 阶段3 放回海绵；默认跟随 ``grasp_sponge``。
        ``gripper_enabled``: 轨迹回放期间是否按录制值控制夹爪，默认
        False（保护海绵的 force=1 低力夹持）。
        """
        trajectory_name = kwargs.get(
            "trajectory", kwargs.get("name", trajectory_name)
        )
        speed = float(speed)
        home_speed = int(home_speed)
        return_home = _as_bool(return_home, default=True)
        gripper_enabled = _as_bool(gripper_enabled, default=False)
        grasp_sponge = _as_bool(grasp_sponge, default=True)
        if put_back is None:
            put_back = grasp_sponge
        else:
            put_back = _as_bool(put_back, default=True)

        home_pose = self.config.get_pose("home", side=self.ARM_SIDE)
        if return_home and not home_pose:
            return SkillResult(
                ok=False,
                code="HOME_POSE_MISSING",
                message="配置中没有右臂 home 位姿，已停止执行",
                recoverable=False,
            )

        stages = []

        if grasp_sponge:
            error = self._grasp_sponge()
            if error:
                return SkillResult(
                    ok=False,
                    code=error,
                    message=f"海绵抓取失败（{error}），已停止，不执行擦桌",
                    recoverable=True,
                    data={"arm": self.ARM_SIDE, "stage": "grasp"},
                )
            stages.append("grasp")

            if not self._return_home_after_grasp():
                return SkillResult(
                    ok=False,
                    code="GRASP_RETURN_HOME_FAILED",
                    message="海绵已抓取，但沿抓取逆路径回 home 失败，已停止，不执行擦桌",
                    recoverable=True,
                    data={"arm": self.ARM_SIDE, "stages": stages},
                )
            stages.append("grasp_return_home")

        if not self._play_trajectory(
            trajectory_name, speed, gripper_enabled, skip_ranges
        ):
            return SkillResult(
                ok=False,
                code="WIPE_TRAJECTORY_FAILED",
                message=f"擦桌子轨迹 {trajectory_name} 执行失败，未继续后续阶段",
                recoverable=True,
                data={"arm": self.ARM_SIDE, "trajectory": trajectory_name},
            )
        stages.append("wipe")

        if put_back:
            if not self._put_back_sponge():
                return SkillResult(
                    ok=False,
                    code="PUT_BACK_FAILED",
                    message="擦桌子轨迹已完成，但放回海绵失败",
                    recoverable=True,
                    data={"arm": self.ARM_SIDE, "stages": stages},
                )
            stages.append("put_back")

        returned_home = False
        if return_home:
            cprint("[wipe_table] 右臂回 home", "cyan")
            returned_home = self.context.arm(self.ARM_SIDE).move_to_named_pose(
                home_pose, speed=home_speed
            )
            if not returned_home:
                return SkillResult(
                    ok=False,
                    code="HOME_RETURN_FAILED",
                    message="擦桌子流程已完成，但右臂回 home 失败",
                    recoverable=True,
                    data={"arm": self.ARM_SIDE, "stages": stages,
                          "returned_home": False},
                )

        cprint("[wipe_table] 擦桌子流程完成", "green")
        return SkillResult(
            ok=True,
            code="SUCCESS",
            message="擦桌子流程执行完成" + (
                "，右臂已回 home" if returned_home else ""
            ),
            data={
                "arm": self.ARM_SIDE,
                "trajectory": trajectory_name,
                "stages": stages,
                "returned_home": returned_home,
            },
        )


def _as_bool(value, default):
    """JSON/命令行布尔兼容：接受字符串形式。"""
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    if value is None:
        return default
    return bool(value)
