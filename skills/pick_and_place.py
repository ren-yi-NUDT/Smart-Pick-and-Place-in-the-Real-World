import random
from termcolor import cprint
from skills.base import Skill, register_skill


@register_skill("pick_and_place")
class PickAndPlaceSkill(Skill):
    """Main pick-and-place pipeline: detect → grasp → place.

    For ``container:"person"``, both arms use their side-specific named-pose
    handover path.
    """

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def execute(self, **kwargs):
        if kwargs.get("object") and kwargs.get("container"):
            json_data = kwargs
        else:
            json_data = self.json_parser.get_command()
        if json_data is None:
            cprint("未收到有效的JSON输入", "red")
            return False

        cprint(f"=================== 1. Get JSON input: {json_data} ===================", "cyan")

        obj, container = json_data.get("object"), json_data.get("container")
        side = json_data.get("side", "left")
        location = json_data.get("location", "desk_front")
        direct_throw = bool(json_data.get("direct_throw", False))
        if obj is None or container is None:
            cprint("JSON输入缺少必需字段 (object 或 container)", "red")
            return False

        cprint(f"=================== 2. Parse input: [{side}] Grasp {obj} and place it in the {container} ===================", "cyan")

        # ---- Grasp phase ----
        if obj.lower() in ["user", "user_hand", "hand"]:
            check = self._receive_from_user(side)
        else:
            check = self._visual_grasp_phase(obj, side=side, location=location)

        if not check:
            return False

        cprint("G=================== 5. Successfully completed the grasping task ===================", "green")

        # ---- Placement phase ----
        if container.lower() == "person":
            return self._place_handover_person(side)

        if container.lower() in ["trash", "垃圾桶", "garbage", "bin"]:
            return self._place_trash(side, direct_throw=direct_throw)

        if container.lower() in ["desk", "桌子", "table"]:
            if side == "right" and not self._handover_right_to_left():
                return False
            return self._place_desk()

        # Drawer / cabinet — narrow receptacle, only right gripper fits.
        # Either arm can grasp; left arm must hand over to right before placing.
        if self._is_right_fixed_placement(container):
            if side == "right":
                return self._place_predefined(container)
            if not self._delegate_to_left_arm(container):
                return False
            return self._place_predefined(container)

        # Visual placement (bowls, plates, etc.) — uses left-arm camera.
        # Right arm must hand over to left arm first.
        if side == "right":
            if not self._handover_right_to_left():
                return False
            return self._place_visual(container)

        return self._place_visual(container)

    # ------------------------------------------------------------------
    # Placement routing helpers
    # ------------------------------------------------------------------
    _DRAWER_KEYWORDS = ("drawer", "cabinet", "抽屉", "柜")

    def _is_right_fixed_placement(self, container):
        """Check if container is a right-arm fixed placement target (drawer/cabinet).
        All drawer-like containers route to the canonical `drawer_1_placement` pose.
        """
        c = container.lower()
        return any(k in c for k in self._DRAWER_KEYWORDS)

    def _place_predefined(self, pose_name):
        """Right arm moves to a predefined pose and releases the object.
        All drawer/cabinet keywords normalize to `drawer_1_placement`.
        """
        right_cfg = self.config.get_arm_config("right")
        if any(k in pose_name.lower() for k in self._DRAWER_KEYWORDS):
            target_pose_name = "drawer_1_placement"
        else:
            target_pose_name = pose_name

        cprint(f"P=================== Right arm fixed placement: {pose_name} → {target_pose_name} ===================", "cyan")

        pose = right_cfg.get(target_pose_name)
        if pose is None:
            cprint(f"P=================== Pose '{target_pose_name}' not found in right arm config ===================", "red")
            return False

        if not self.place_pipeline.place_at_named_pose(
                target_pose_name, side="right"):
            return False
        cprint(f"P=================== Fixed placement to {target_pose_name} done ===================", "green")
        return True
    def _delegate_to_left_arm(self, container):
        """Replay the latest recorded left→right dual-arm handover trajectory.

        The latest recording contains the synchronized arm motion and the
        timed gripper events (right close, left open, then both arms home).
        ``container`` is retained for callers that use the older API.
        """
        cprint(
            f"D=================== Two-arm handover (latest recording, container={container}) ===================",
            "cyan",
        )
        try:
            ok = self.handover_pipeline.run(
                mode="dual", speed=0.9, require_confirmation=False,
                direction="left_to_right",
            )
            if ok:
                cprint(
                    "D=================== Latest handover done (object now in right gripper at home) ===================",
                    "green",
                )
            return bool(ok)
        except Exception as exc:
            cprint(f"D=================== Latest handover failed: {exc} ===================", "red")
            return False

    def _handover_right_to_left(self):
        """Right→left handover: same recorded trajectory as left→right,
        gripper event roles swapped via ``direction`` (left closes to
        receive, right opens to release).

        Used when right arm grasped the object and left arm needs to place it
        (side=right + visual container, or side=right + person).
        """
        cprint("H=================== Right→left handover (recorded replay) ===================", "cyan")
        try:
            ok = self.handover_pipeline.run(
                mode="dual", speed=0.9, require_confirmation=False,
                direction="right_to_left",
            )
            if ok:
                cprint(
                    "H=================== Handover done (object now in left gripper at home) ===================",
                    "green",
                )
            return bool(ok)
        except Exception as exc:
            cprint(f"H=================== Right→left handover failed: {exc} ===================", "red")
            return False

    # ------------------------------------------------------------------
    def _visual_grasp_phase(self, obj, side="left", location="desk_front"):
        """Unified visual grasp for both arms."""
        return self.visual_grasp(obj, side=side, location=location)

    def _receive_from_user(self, side="left"):
        """Receive object from user at handover pose."""
        return self.handover_pipeline.run(
            mode="receive_user", side=side,
            wait_seconds=4.0, retry_wait_seconds=3.0,
        )

    # ------------------------------------------------------------------
    # Placement phase variants
    # ------------------------------------------------------------------
    def _place_handover_person(self, side="left"):
        """Deliver object to person; both arms share the same named-pose flow."""
        cprint("H=================== Handover mode detected: delivering to person ===================", "cyan")
        return self.handover_pipeline.run(
            mode="user_release", side=side, speed=15, release_wait=2.0,
        )

    def _place_trash(self, side="left", direct_throw=False):
        """Throw object to trash.

        ``direct_throw=True`` 时由抓取臂用自己的 throw_to_trash_pose 直接扔
        （左右臂都录制了该位姿）；否则左臂先双臂交接给右臂再扔。
        """
        cprint("T=================== Trash mode detected ===================", "cyan")

        if direct_throw:
            return self._single_arm_trash(side)
        if side == "left":
            return self._dual_arm_trash()
        else:
            return self._single_arm_trash(side)

    def _place_desk(self):
        """Place object on desk."""
        cprint("D=================== Desk placement mode detected: placing on desk ===================", "cyan")
        selected = random.choice(["desk_pose_1", "desk_pose_2", "desk_pose_3"])
        if not self.place_pipeline.place_at_named_pose(selected, side="left"):
            return False
        cprint("D=================== 5. Successfully completed the desk placement task ===================", "green")
        return True

    def _place_visual(self, container):
        """Vision-based placement into detected container."""
        return self.place_pipeline.run(
            container=container, side="left", object_name=None,
        )

    # ------------------------------------------------------------------
    # Dual-arm trash: left arm transfers to right arm, right arm throws
    # ------------------------------------------------------------------
    def _dual_arm_trash(self):
        """Left→right handover then right arm throws to trash.

        Reuses the validated 4-step handover sequence in `_delegate_to_left_arm`,
        then right arm carries object from home to throw_to_trash_pose and releases.
        """
        cprint("T=================== Dual-arm trash: left → right handover, right throws ===================", "cyan")
        # Phase 1: Validated 4-step handover (ends with both arms at home, object in right gripper)
        if not self._delegate_to_left_arm(container="trash"):
            cprint("T=================== Handover failed ===================", "red")
            return False

        # Phase 2: Right arm throws to trash
        right_cfg = self.config.get_arm_config("right")
        throw_pose = right_cfg.get("throw_to_trash_pose")
        if throw_pose is None:
            cprint("T=================== throw_to_trash_pose not found for right arm ===================", "red")
            return False

        cprint("T=================== Right arm: home → throw_to_trash_pose (release) → home ===================", "cyan")
        if not self.place_pipeline.place_at_named_pose(
                "throw_to_trash_pose", side="right"):
            return False

        cprint("T=================== Dual-arm trash done ===================", "green")
        return True

    def _single_arm_trash(self, side="right"):
        """Arm throws trash directly with its own throw_to_trash_pose."""
        cprint(f"T=================== Single arm trash: {side} throws directly ===================", "cyan")

        arm_cfg = self.config.get_arm_config(side)
        throw_pose = arm_cfg.get("throw_to_trash_pose")
        if throw_pose is None:
            cprint(f"T=================== throw_to_trash_pose not found for {side} arm ===================", "red")
            return False

        if not self.place_pipeline.place_at_named_pose(
                "throw_to_trash_pose", side=side):
            return False

        cprint("T=================== 5. Successfully completed the single-arm trash task ===================", "green")
        return True
