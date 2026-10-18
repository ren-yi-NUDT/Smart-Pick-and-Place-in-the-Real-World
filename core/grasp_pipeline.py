#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉抓取管线 — AnyGrasp 候选 → 打分 → Twin 规划 → 执行。

从原项目 (Smart-Pick-and-Place-in-the-Real-World 的 core/grasp_pipeline.py) 复刻到
单臂 RM65 平台。与原版的差异（都是本平台的硬件/依赖现实决定的，其余逻辑逐行保留）：

1. 去掉 ``side`` 双臂参数：单臂，夹爪是 Robotiq 2F-85，走原版 **left** 分支的约定
   （夹爪座相对旧灵巧手约定旋转 180°，即 ``hand_convention @ diag([-1,-1,1,1])``）。
2. 7 自由度 → 6 自由度：位姿校验 J1..J7 → J1..J6；腕部转角限幅用 J6（waypoint[5]）。
3. sim/suction 分支删除（本平台没有 sim 后端，``config.sim_mode`` 恒为 False）。
4. slender（细长物体专用候选生成）和 VLM grounding 未随行移植 —— 本平台 perception
   没有 is_slender_object / vlm 没有 ground_object，``use_vlm_grounding=True`` 会
   显式报错而不是半路 AttributeError。
5. 抓住后的"补夹"用 ``hand.hold(force)`` 而不是原版的 ``hand.close(force)``：
   本平台夹爪服务的 close() 会一路合到机械限位，会把果子挤出去
   （见 core/gripper.py:hold 的注释）；hold() 停在当前开度只加力。

标定链（用户关注点，出抓偏先查这两条）：
  - ``T_base_to_cam``：TF base_link → cam_link_grasp（腕上相机的手眼外参，观测时现取）
  - ``T_hand_effector_to_arm_endlink``：TF gripper_endeffector → link_6（夹爪到末端连杆）
"""

import numpy as np


class GraspPipeline:
    """Grasp chain composed around an existing Skill.

    Attribute access falls through to the owning skill, so moved methods
    keep using self.config / self.perception / self.control_arm etc.
    unchanged.
    """

    def __init__(self, skill):
        self._skill = skill

    def __getattr__(self, name):
        try:
            skill = object.__getattribute__(self, "_skill")
        except AttributeError:
            raise AttributeError(name)
        return getattr(skill, name)

    def _set_runtime_attr(self, name, value):
        """Store cross-pipeline runtime state on the owning Skill."""
        setattr(self._skill, name, value)

    def _set_grasp_failure(self, code, message):
        self._set_runtime_attr(
            "_last_grasp_failure", {"code": str(code), "message": str(message)}
        )

    def _grasp_arm_speed(self, scoring, key, default, scale=None):
        """Scale grasp-only arm motion without changing gripper speed."""
        import math

        if scale is None:
            scale = self.config.shared.get("grasp_arm_speed_scale", 2.0)
        scale = float(scale)
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError(f"invalid grasp arm speed scale: {scale}")
        return min(100, max(1, int(round(float(scoring.get(key, default)) * scale))))

    # ------------------------------------------------------------------
    # Unified visual grasp pipeline
    # ------------------------------------------------------------------
    def _grasp_observation_poses(self, location=None, observation_pose=None):
        if isinstance(observation_pose, dict):
            required = {f"J{i}" for i in range(1, 7)}
            if required.issubset(observation_pose):
                # A local pre-action is only a fast first view.  Keep the
                # configured views as an accuracy fallback when its image is
                # occluded, at the image edge, or yields no grasp points.
                poses = [("current", observation_pose)]
                defaults = self.config.default_traj_js
                poses.extend(
                    (name, pose) for name, pose in defaults.items()
                    if name != "current" and "grasp" in name
                    and isinstance(pose, dict)
                )
                return poses
        poses = self.config.default_traj_js
        return [
            (name, pose) for name, pose in poses.items()
            if "grasp" in name and isinstance(pose, dict)
        ]

    @staticmethod
    def _normalise(values):
        values = np.asarray(values, dtype=float)
        if not len(values):
            return values
        lo, hi = float(np.min(values)), float(np.max(values))
        if hi - lo < 1e-9:
            return np.ones_like(values)
        return (values - lo) / (hi - lo)

    def _build_grasp_candidates(self, grasp_data):
        """Convert camera-frame AnyGrasp results into base-frame candidates."""
        from core.transforms import self_rotation_np

        T_base_to_cam = getattr(self, "T_base_to_cam", None)
        T_hand_to_end = getattr(self, "T_hand_effector_to_arm_endlink", None)
        if T_base_to_cam is None or T_hand_to_end is None:
            raise RuntimeError("标定矩阵未保存（save_current_transformation 未调用）")
        candidates = []
        for index, data in enumerate(grasp_data or []):
            try:
                T_grasp = np.eye(4)
                T_grasp[:3, :3] = np.asarray(data["rotation_matrix"], dtype=float)
                T_grasp[:3, 3] = np.asarray(data["trans"], dtype=float)
                hand_convention = self_rotation_np(np.array([
                    [0, 1, 0, 0], [-1, 0, 0, 0],
                    [0, 0, 1, 0], [0, 0, 0, 1],
                ], dtype=float))
                # 原版 left 分支: Robotiq 夹爪座相对旧灵巧手约定旋转 180°。
                hand_convention = hand_convention @ np.diag([-1., -1., 1., 1.])
                T_world_hand = T_base_to_cam @ (T_grasp @ hand_convention)
                if T_world_hand[:3, 0][0] < 0:
                    T_world_hand = T_world_hand @ np.diag([-1., -1., 1., 1.])
                candidate = {
                    "index": index,
                    "pose": T_world_hand,
                    "original_pose": data,
                    "anygrasp_score": float(data.get("score", 0.0)),
                    "width_m": data.get("width", data.get("gripper_width")),
                    "height_m": data.get("height", data.get("gripper_height")),
                    "T_hand_to_end": T_hand_to_end,
                    # Keep the intermediate geometry in the candidate so a
                    # real-hardware run can distinguish camera/depth error
                    # from the fixed hand-to-arm-end offset.
                    "camera_translation_m": [
                        float(value) for value in T_grasp[:3, 3]
                    ],
                    "hand_position_base_m": [
                        float(value) for value in T_world_hand[:3, 3]
                    ],
                    "hand_to_end_translation_m": [
                        float(value) for value in T_hand_to_end[:3, 3]
                    ],
                }
                camera_ray = np.asarray(data["trans"], dtype=float).reshape(3)
                camera_ray /= max(float(np.linalg.norm(camera_ray)), 1e-9)
                raw_rotation = np.asarray(
                    data["rotation_matrix"], dtype=float
                ).reshape(3, 3)
                candidate["camera_ray_base"] = [
                    float(value)
                    for value in T_base_to_cam[:3, :3] @ camera_ray
                ]
                candidate["camera_ray_alignment"] = float(
                    np.dot(raw_rotation[:, 0], camera_ray)
                )
                # Keep optional labels and all original metadata for logging.
                if "label" in data:
                    candidate["label"] = data["label"]
                candidates.append(candidate)
            except (KeyError, TypeError, ValueError) as exc:
                from termcolor import cprint
                cprint(f"[grasp] invalid AnyGrasp candidate {index}: {exc}", "yellow")

        scores = self._normalise([c["anygrasp_score"] for c in candidates])
        for candidate, score in zip(candidates, scores):
            candidate["anygrasp_normalized"] = float(score)
        return candidates

    def _grasp_target_config(
        self, candidate, obs_pose, high_pregrasp_offset_m=None
    ):
        from scipy.spatial.transform import Rotation as R

        scoring = self.config.get_grasp_scoring()
        pose = np.asarray(candidate["pose"], dtype=float)
        # Build a staged approach in the base frame.  The old two-point path
        # went directly from the observation pose to a 2 cm pre-grasp point;
        # for some IK branches this makes the shoulder/wrist reconfigure while
        # the tool is already close to the object.  Keep the grasp orientation
        # fixed and do the reconfiguration at a higher clearance instead.
        high_pre = pose.copy()
        high_offset = (
            float(scoring.get("high_pregrasp_offset_m", 0.08))
            if high_pregrasp_offset_m is None
            else float(high_pregrasp_offset_m)
        )
        high_pre[2, 3] += high_offset
        low_pre = pose.copy()
        low_offset = float(scoring.get("pregrasp_offset_m", 0.02))
        low_pre[2, 3] += low_offset
        execution = pose.copy()
        T_hand_to_end = candidate["T_hand_to_end"]
        high_pre = high_pre @ T_hand_to_end
        low_pre = low_pre @ T_hand_to_end
        execution = execution @ T_hand_to_end

        # Empirical correction for the repeatable physical X error observed
        # between the commanded end-effector pose and the real grasp point.
        # Apply it after hand->end conversion so the offset is expressed in
        # the arm-base frame, not in the rotated grasp/hand frame.
        execution_offset = np.asarray(
            scoring.get("execution_offset_base_m", [0.0, 0.0, 0.0]),
            dtype=float,
        )
        if execution_offset.shape != (3,) or not np.all(np.isfinite(execution_offset)):
            raise ValueError(
                f"invalid execution_offset_base_m: {execution_offset}"
            )
        for target in (high_pre, low_pre, execution):
            target[:3, 3] += execution_offset
        candidate["execution_offset_base_m"] = [
            float(value) for value in execution_offset
        ]
        candidate["target_hand_position_base_m"] = [
            float(value) for value in pose[:3, 3]
        ]
        candidate["target_end_position_base_m"] = [
            float(value) for value in execution[:3, 3]
        ]

        def pose7(matrix):
            p = matrix[:3, 3]
            q = R.from_matrix(matrix[:3, :3]).as_quat()
            return [float(v) for v in (*p, *q)]

        current_js = [float(v) * np.pi / 180.0 for v in obs_pose.values()]
        result = {
            # Observation -> high pre-grasp -> low pre-grasp -> grasp.
            # Twin's trajectory_generation2 interpolates each consecutive
            # pair and checks every generated waypoint.
            "target_pose": [pose7(high_pre), pose7(low_pre), pose7(execution)],
            "current_js": current_js,
            "struct": self.config.get_arm_config().get("twin_struct", "rm65"),
            "interval_threshold": float(
                scoring.get("approach_interval_m", 0.025)
            ),
            "rotation_interval_rad": float(
                scoring.get("approach_rotation_interval_rad", 0.15)
            ),
            "joint_step_limit_rad": float(
                scoring.get("joint_step_limit_rad", 0.12)
            ),
            # Pre-grasp waypoints are clearance/interpolation anchors.  The
            # final contact waypoint remains checked by Twin and by the
            # physical gripper, while this task may explicitly allow a small
            # IK residual on the non-contact waypoints.
            "xyz_threshold": float(
                candidate.get(
                    "grasp_xyz_threshold_m",
                    self.config.shared.get("grasp_xyz_threshold_m", 0.015),
                )
            ),
            "rpy_threshold": float(
                candidate.get(
                    "grasp_rpy_threshold_rad",
                    self.config.shared.get("grasp_rpy_threshold_rad", 0.05),
                )
            ),
        }
        return result

    def _plan_grasp_candidate(
        self,
        candidate,
        obs_pose,
        max_gripper_axis_rotation_deg=None,
    ):
        """Use Twin once as a reachability test and cache its trajectory."""
        from termcolor import cprint
        scoring = self.config.get_grasp_scoring()
        # 2026-10-10 香蕉回合: 没有任何调用方传过这个限位, J6 翻腕检查从未生效,
        # IK 选中翻 180° 的分支, 接近途中整条臂跟着扭过去。默认从配置取。
        if max_gripper_axis_rotation_deg is None:
            max_gripper_axis_rotation_deg = scoring.get(
                "max_gripper_axis_rotation_deg")
        preferred_high = float(scoring.get("high_pregrasp_offset_m", 0.08))
        # Keep the staged approach, but adapt its clearance to the arm's
        # actual workspace.  A fixed high point can be unreachable for an
        # object near the edge of the workspace; reducing clearance is safer
        # than reverting to a direct object-level move. Every attempt still
        # runs Twin's full waypoint checks.
        offsets = []
        for offset in (preferred_high, 0.06, 0.04):
            if offset > float(scoring.get("pregrasp_offset_m", 0.02)):
                if not any(abs(offset - old) < 1e-6 for old in offsets):
                    offsets.append(offset)
        last_error = "unreachable"
        try:
            for high_offset in offsets:
                # 平行夹爪 180° 对称: pose 和 pose@diag(-1,-1,1,1) 是同一个物理抓取,
                # 但腕部 J6 差 ~180°。_make_candidates 的 X 轴符号启发式可能挑到
                # 离观察位姿远的那支 (2026-10-14 苹果: 156° vs 24°), 限位拒绝后
                # 用 jaw 翻转的等价位姿重试一次, 而不是直接否掉候选。
                base_pose = np.asarray(candidate["pose"], dtype=float)
                for jaw_flipped in (False, True):
                    if jaw_flipped:
                        candidate["pose"] = base_pose @ np.diag(
                            [-1., -1., 1., 1.])
                        candidate["jaw_flipped"] = True
                    cnfg = self._grasp_target_config(
                        candidate, obs_pose,
                        high_pregrasp_offset_m=high_offset,
                    )
                    rsp = self.twin.generate_trajectory2(cnfg)
                    if not bool(rsp.get("value")):
                        last_error = rsp.get("info", "unreachable")
                        continue
                    trajectory = rsp.get("info", {}).get("trajectory")
                    if not trajectory:
                        last_error = "Twin returned no trajectory"
                        continue
                    candidate["trajectory"] = trajectory
                    candidate["grasp_high_pregrasp_offset_m"] = high_offset
                    candidate["twin_reachable"] = 1.0
                    if (
                        max_gripper_axis_rotation_deg is not None
                        and not self._enforce_gripper_axis_rotation_limit(
                            candidate,
                            obs_pose,
                            max_gripper_axis_rotation_deg,
                        )
                    ):
                        last_error = candidate["twin_error"]
                        cprint(
                            f"[grasp] rejected candidate "
                            f"{candidate.get('index', '?')}: {last_error}",
                            "yellow",
                        )
                        continue
                    return True
                candidate["pose"] = base_pose
                candidate.pop("jaw_flipped", None)
            candidate["twin_reachable"] = 0.0
            candidate["twin_error"] = last_error
            cprint(
                f"[grasp] Twin rejected candidate: {last_error}",
                "yellow",
            )
            return False
        except Exception as exc:
            candidate["twin_reachable"] = 0.0
            candidate["twin_error"] = str(exc)
            self._drop_twin()
            cprint(f"[grasp] Twin reachability failed: {exc}", "yellow")
            return False

    def _plan_best_reachable_candidate(
        self, candidates, pose_name, obs_pose,
        max_gripper_axis_rotation_deg=None,
    ):
        """Return the highest-scoring reachable candidate.

        Twin contributes a binary reachability score, so planning candidates
        in their non-Twin score order and stopping at the first success picks
        the same candidate as planning every candidate. A failed physical
        grasp already invalidates the RGB-D frame, so lower-ranked plans would
        not be reused.
        """
        ranked = self._score_grasp_candidates(candidates)
        for candidate in ranked:
            candidate["observation_pose_name"] = pose_name
            if self._plan_grasp_candidate(
                candidate,
                obs_pose,
                max_gripper_axis_rotation_deg=max_gripper_axis_rotation_deg,
            ):
                return candidate
        return None

    @staticmethod
    def _enforce_gripper_axis_rotation_limit(candidate, obs_pose, limit_deg):
        """Reject a planned path whose wrist joint (J6) strays too far from start."""
        import math

        try:
            limit = float(limit_deg)
            start = math.radians(float(obs_pose["J6"]))
            trajectory = candidate["trajectory"]
            if not math.isfinite(limit) or limit < 0.0 or not trajectory:
                raise ValueError("invalid J6 rotation limit or trajectory")

            deltas = []
            for waypoint in trajectory:
                angle = float(waypoint[5])
                if not math.isfinite(angle):
                    raise ValueError("non-finite J6 angle")
                difference = angle - start
                wrapped = math.atan2(math.sin(difference), math.cos(difference))
                deltas.append(abs(math.degrees(wrapped)))
            max_delta = max(deltas)
        except (KeyError, IndexError, TypeError, ValueError):
            limit = float(limit_deg) if isinstance(limit_deg, (int, float)) else -1.0
            max_delta = float("inf")

        candidate["wrist_max_rotation_deg"] = max_delta
        candidate["wrist_rotation_limit_deg"] = limit
        if max_delta <= limit + 1e-9:
            return True

        candidate["twin_reachable"] = 0.0
        candidate["twin_error"] = (
            f"J6 rotation {max_delta:.2f} deg exceeds {limit:.2f} deg"
        )
        return False

    def _score_grasp_candidates(self, candidates):
        """Combine AnyGrasp, Twin, width and approach-angle preferences."""
        import math

        cfg = self.config.get_grasp_scoring()
        weights = cfg.get("weights", {})
        preferred_width = float(cfg.get("preferred_width_m", 0.045))
        width_tolerance = max(float(cfg.get("width_tolerance_m", 0.035)), 1e-6)
        preferred_height = float(cfg.get("preferred_gripper_height_m", 0.03))
        height_tolerance = max(float(cfg.get("height_tolerance_m", 0.01)), 1e-6)
        max_width = float(cfg.get("max_width_m", 0.085))
        preferred_axis = np.asarray(
            cfg.get("preferred_approach_axis_base", [0., 0., -1.]), dtype=float
        )
        preferred_axis /= max(np.linalg.norm(preferred_axis), 1e-9)
        local_axis = np.asarray(cfg.get("approach_axis_local", [0., 0., 1.]), dtype=float)
        local_axis /= max(np.linalg.norm(local_axis), 1e-9)

        for candidate in candidates:
            width = candidate.get("width_m")
            if width is None:
                width_score = 0.5  # Older server protocol did not return width.
            else:
                width = float(width)
                if width > 1.0:  # tolerate SDKs returning millimetres
                    width /= 1000.0
                candidate["width_m"] = width
                width_score = math.exp(-abs(width - preferred_width) / width_tolerance)
                if width > max_width:
                    width_score = 0.0
            height = candidate.get("height_m")
            if height is None:
                length_score = 0.5
            else:
                height = float(height)
                if height > 1.0:
                    height /= 1000.0
                candidate["height_m"] = height
                length_score = math.exp(
                    -abs(height - preferred_height) / height_tolerance
                )
            approach_axis = candidate["pose"][:3, :3] @ local_axis
            angle_score = max(0.0, float(np.dot(approach_axis, preferred_axis)))
            candidate["width_score"] = float(width_score)
            candidate["length_score"] = float(length_score)
            candidate["angle_score"] = float(angle_score)
            candidate["composite_score"] = float(
                float(weights.get("anygrasp", 0.45)) * candidate.get("anygrasp_normalized", 0.0)
                + float(weights.get("twin", 0.30)) * candidate.get("twin_reachable", 0.0)
                + float(weights.get("width", 0.15)) * width_score
                + float(weights.get("length", 0.05)) * length_score
                + float(weights.get("angle", 0.10)) * angle_score
            )
        return sorted(candidates, key=lambda c: c["composite_score"], reverse=True)

    def _recover_grasp_failure(self, obs_pose, arm_speed_scale=None):
        """Return the arm to a known observation pose and open the gripper."""
        from termcolor import cprint
        try:
            scoring = self.config.get_grasp_scoring()
            if obs_pose:
                self.arm.move_to_named_pose(
                    obs_pose,
                    speed=self._grasp_arm_speed(
                        scoring, "grasp_recovery_speed", 10, arm_speed_scale,
                    ),
                )
            self.hand.open()
        except Exception as exc:
            cprint(f"[grasp] recovery failed: {exc}", "red")

    def _execute_scored_grasp(
        self, candidate, obs_pose, hold_after_grasp=False,
        verify_lift=True, verify_contact=True, arm_speed_scale=None,
    ):
        """Execute a cached Twin plan and verify contact before/after lifting.

        ``hold_after_grasp`` is used by handover state machines: after the
        gripper confirms contact, leave the arm at the grasp pose so a
        separately recorded handover trajectory can continue from there.
        The default keeps the historical behavior of returning to the
        observation pose and verifying the lift.
        """
        import time
        from termcolor import cprint
        scoring = self.config.get_grasp_scoring()
        close_force = int(scoring.get("gripper_close_force", 20))
        try:
            # Never send a grasp trajectory while the fingers may still be
            # closed around a previous object or may have stopped mid-motion.
            # Re-open and verify before every candidate, including runtime
            # retries after a missed grasp.
            if not self.hand.is_fully_open():
                cprint("[grasp] gripper not open; opening before grasp", "yellow")
                self.hand.open()
                time.sleep(0.3)
            if not self.hand.is_fully_open():
                raise RuntimeError("gripper is not fully open before grasp")
            # Twin returns radians; arm services use degrees.
            trajectory = np.asarray(candidate["trajectory"], dtype=float) * 180.0 / np.pi
            if not self.arm.execute_trajectory(
                trajectory,
                speed=self._grasp_arm_speed(
                    scoring, "grasp_trajectory_speed", 12, arm_speed_scale,
                ),
            ):
                raise RuntimeError(
                    getattr(self.arm, "last_error", None)
                    or "arm trajectory execution failed"
                )
            cprint(
                f"[grasp] geometry: cam_trans="
                f"{candidate.get('camera_translation_m')} "
                f"hand_base="
                f"{candidate.get('target_hand_position_base_m', candidate.get('hand_position_base_m'))} "
                f"end_target="
                f"{candidate.get('target_end_position_base_m')} "
                f"hand_to_end_t={candidate.get('hand_to_end_translation_m')}",
                "cyan",
            )
            close_resp = self.hand.close(
                force=close_force,
                speed=int(scoring.get("gripper_close_speed", 20)),
                soft=True,
            )
            time.sleep(0.3)
            response_info = close_resp.get("info", {}) if isinstance(close_resp, dict) else {}
            if not isinstance(response_info, dict):
                response_info = {}
            detected = close_resp.get("object_detected")
            if detected is None:
                # The real gripper service may wrap its contact result
                # under ``info``.
                detected = response_info.get("object_detected")
            if verify_contact and detected is None:
                detected = self.hand.is_grasping(force=close_force)
            if verify_contact and not bool(detected):
                cprint("[grasp] no object detected after close", "yellow")
                self._set_grasp_failure(
                    "GRASP_CONTACT_FAILED", "夹爪闭合后未检测到物体"
                )
                self._recover_grasp_failure(obs_pose, arm_speed_scale)
                return False

            # Soft close holds at force=0 (server-side design to protect
            # delicate objects). 原版这里用 close(force) 补夹 —— 本平台夹爪的
            # close() 会一路合到机械限位把果子挤出去, 必须用 hold() 停在当前
            # 开度只加力 (见 core/gripper.py:hold)。
            held = self.hand.hold(
                force=int(scoring.get("gripper_hold_force", 80)),
            )
            if isinstance(held, dict) and held.get("value") is False:
                raise RuntimeError("gripper hold failed after soft close")
            time.sleep(0.3)

            if not hold_after_grasp and obs_pose:
                # Retrace the Twin-validated path for every visual grasp. It
                # first backs away from the contact point through the same
                # collision-checked waypoints, then returns to observation.
                if not self.arm.execute_trajectory(
                    trajectory[::-1],
                    speed=self._grasp_arm_speed(
                        scoring, "grasp_post_lift_speed", 15, arm_speed_scale,
                    ),
                ):
                    raise RuntimeError(
                        getattr(self.arm, "last_error", None)
                        or "post-grasp safe retreat failed"
                    )
                cprint(
                    "[grasp] safe retreat: retraced Twin path before return",
                    "cyan",
                )
            if hold_after_grasp:
                cprint(
                    "[grasp] object confirmed; holding at grasp pose",
                    "green",
                )
            elif (
                verify_lift
                and scoring.get("verify_lift", True)
                and not self.hand.is_grasping(
                    force=int(scoring.get("gripper_close_force", 20))
                )
            ):
                cprint("[grasp] object lost after lift", "yellow")
                self._set_grasp_failure(
                    "GRASP_LOST_AFTER_LIFT", "物体在抓取后安全撤离时脱落"
                )
                self._recover_grasp_failure(obs_pose, arm_speed_scale)
                return False
            self._set_runtime_attr("_last_successful_grasp_candidate", candidate)
            cprint(
                f"[grasp] selected candidate {candidate['index']} "
                f"score={candidate['composite_score']:.3f} "
                f"(AnyGrasp={candidate.get('anygrasp_normalized', 0):.3f}, "
                f"Twin={candidate.get('twin_reachable', 0):.0f}, "
                f"width={candidate.get('width_score', 0):.3f}, "
                f"length={candidate.get('length_score', 0):.3f}, "
                f"angle={candidate.get('angle_score', 0):.3f})",
                "green",
            )
            return True
        except Exception as exc:
            cprint(f"[grasp] execution failed: {exc}", "red")
            self._set_grasp_failure(
                "GRASP_EXECUTION_FAILED", f"抓取执行失败：{exc}"
            )
            self._recover_grasp_failure(obs_pose, arm_speed_scale)
            return False

    def visual_grasp(
        self,
        object_name,
        location=None,
        hold_after_grasp=False,
        observation_pose=None,
        use_vlm_grounding=False,
        verify_lift=True,
        verify_contact=True,
        min_camera_ray_alignment=None,
        max_gripper_axis_rotation_deg=None,
        arm_speed_scale=None,
    ):
        """RGB-D 抓取统一入口，所有 skill 共用。

        When ``hold_after_grasp`` is true, a successful gripper contact leaves
        the arm at its grasp pose for a subsequent handover trajectory.
        """
        from termcolor import cprint
        self._set_runtime_attr("_last_grasp_failure", None)
        # VLM grounding / slender 专用候选生成未随行移植 (见模块 docstring)。
        if use_vlm_grounding:
            self._set_grasp_failure(
                "GRASP_GROUNDING_UNAVAILABLE",
                "VLM grounding 未移植到单臂平台, use_vlm_grounding 需为 False",
            )
            cprint("[grasp] VLM grounding 未移植, 按 YOLO 框过滤继续", "yellow")
        # Do not expose a previous task's candidate if this invocation fails.
        self._set_runtime_attr("_last_successful_grasp_candidate", None)
        self._set_runtime_attr("_last_grasp_candidates", [])
        for pose_name, obs_pose in self._grasp_observation_poses(
            location, observation_pose=observation_pose
        ):
            try:
                if pose_name != "current":
                    scoring = self.config.get_grasp_scoring()
                    if not self.control_arm(
                        pose_type=pose_name,
                        speed=self._grasp_arm_speed(
                            scoring, "grasp_observation_speed", 15,
                            arm_speed_scale,
                        ),
                    ):
                        self._set_grasp_failure(
                            "GRASP_OBSERVATION_POSE_FAILED",
                            f"无法到达抓取观察位 {pose_name}",
                        )
                        continue
                self.control_hand(cmd_type="open")
                rgb, depth = self.get_camera_obs()
                self.rgb, self.depth = rgb, depth
                self.save_current_transformation()
                camera = self.camera
                depth_scale = getattr(camera, "depth_scale", None)
                cprint(
                    f"[grasp] camera calibration: serial="
                    f"{getattr(camera, 'serial', 'n/a')} "
                    f"depth_scale_m={depth_scale if depth_scale is not None else 0.001}",
                    "cyan",
                )
                raw = self.perception.detect_grasps(
                    rgb, depth, class_name=object_name,
                    raw_depth=getattr(camera, "raw_depth", None),
                )
                if not raw:
                    cprint(f"[grasp] no AnyGrasp candidates at {pose_name}", "yellow")
                    self._set_grasp_failure(
                        "GRASP_NO_CANDIDATES",
                        f"AnyGrasp 在观察位 {pose_name} 未生成抓取候选",
                    )
                    continue
                detector_prompts = [object_name]
                cprint(
                    f"[grasp] YOLO prompts: {detector_prompts}",
                    "cyan",
                )
                # 原版语义: 用目标名直接过滤 (YOLO-World 开放词汇, 腕上近景也认)。
                filtered = self.perception.filter_grasps_by_detection(
                    raw, rgb, class_name=object_name,
                    vis=True,
                )
                candidates = self._build_grasp_candidates(filtered)
                if min_camera_ray_alignment is not None:
                    threshold = float(min_camera_ray_alignment)
                    candidates = [
                        candidate for candidate in candidates
                        if candidate.get("camera_ray_alignment", -1.0) >= threshold
                    ]
                    if not candidates:
                        cprint(
                            f"[grasp] no candidate approaches the verified "
                            f"surface within camera-ray threshold {threshold:.2f}",
                            "yellow",
                        )
                        self._set_grasp_failure(
                            "GRASP_ALIGNMENT_FAILED",
                            f"抓取候选与目标表面的接近方向均低于阈值 {threshold:.2f}",
                        )
                if not candidates:
                    self._set_grasp_failure(
                        "GRASP_NO_MATCHING_CANDIDATES",
                        f"观察位 {pose_name} 没有与目标匹配的抓取候选",
                    )
                    continue
                candidate = self._plan_best_reachable_candidate(
                    candidates,
                    pose_name,
                    obs_pose,
                    max_gripper_axis_rotation_deg=max_gripper_axis_rotation_deg,
                )
                if candidate is None:
                    cprint(f"[grasp] no Twin-reachable candidate at {pose_name}", "yellow")
                    details = next(
                        (c.get("twin_error") for c in candidates if c.get("twin_error")),
                        "Twin 未返回可达轨迹",
                    )
                    self._set_grasp_failure(
                        "GRASP_PLANNING_FAILED", f"抓取路径规划失败：{details}"
                    )
                    continue
                ranked = self._score_grasp_candidates(candidates)
                self._set_runtime_attr("_last_grasp_candidates", ranked)
                if self._execute_scored_grasp(
                    candidate,
                    obs_pose,
                    hold_after_grasp=hold_after_grasp,
                    verify_lift=verify_lift,
                    verify_contact=verify_contact,
                    arm_speed_scale=arm_speed_scale,
                ):
                    return True
                # A physical failure can move the object. Do not try a second
                # pose computed from this stale RGB-D frame.
                # 原版这里会重置 detector_prompts/target_box 后试下一个观察位;
                # 单臂平台过滤只靠 "object" 框, 无需重置。
            except Exception as exc:
                cprint(f"[grasp] observation '{pose_name}' failed: {exc}", "red")
                self._set_grasp_failure(
                    "GRASP_PIPELINE_ERROR",
                    f"抓取流程在观察位 {pose_name} 失败：{exc}",
                )
                self._recover_grasp_failure(obs_pose, arm_speed_scale)
        cprint("[grasp] all observation poses failed", "red")
        return False

    def _drop_twin(self):
        """Drop the twin client so the owning Skill's lazy property reconnects."""
        self._skill._twin = None
