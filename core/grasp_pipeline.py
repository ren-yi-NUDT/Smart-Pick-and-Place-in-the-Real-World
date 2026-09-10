#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Visual grasp pipeline — AnyGrasp candidates → scoring → Twin planning → execution.

Extracted from skills/base.py so the Skill base class only provides
infrastructure (hardware dispatch, control primitives). The pipeline
delegates infrastructure access back to the owning skill instance, so no
extra sockets or skill instances are created.
"""

import os
import socket
import time


class GraspPipeline:
    """Grasp chain for either arm, composed around an existing Skill.

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
        owner = getattr(self._skill, "skill", self._skill)
        setattr(owner, name, value)

    # ------------------------------------------------------------------
    # Unified visual grasp pipeline
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # Unified visual grasp pipeline
    # ------------------------------------------------------------------
    def _grasp_observation_poses(self, side, location=None, observation_pose=None):
        if isinstance(observation_pose, dict):
            required = {f"J{i}" for i in range(1, 8)}
            if required.issubset(observation_pose):
                return [("current", observation_pose)]
        arm_cfg = self.config.get_arm_config(side)
        poses = arm_cfg.get("default_traj_js", {})
        if side == "right":
            pose = poses.get(location or "desk_front")
            return [(location or "desk_front", pose)] if pose is not None else []
        return [
            (name, pose) for name, pose in poses.items()
            if "grasp" in name and isinstance(pose, dict)
        ]

    @staticmethod
    def _normalise(values):
        import numpy as np
        values = np.asarray(values, dtype=float)
        if not len(values):
            return values
        lo, hi = float(np.min(values)), float(np.max(values))
        if hi - lo < 1e-9:
            return np.ones_like(values)
        return (values - lo) / (hi - lo)

    def _build_grasp_candidates(self, grasp_data, side):
        """Convert camera-frame AnyGrasp results into side-local candidates."""
        import numpy as np
        from core.transforms import self_rotation_np

        T_base_to_cam, T_hand_to_end = self._get_side_transforms(side)
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
                if side == "left":
                    # The left Robotiq mount is rotated 180 degrees relative
                    # to the old dexterous-hand convention.
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
                    "side": side,
                }
                # Keep optional labels and all original metadata for logging.
                if "label" in data:
                    candidate["label"] = data["label"]
                candidates.append(candidate)
            except (KeyError, TypeError, ValueError) as exc:
                from termcolor import cprint
                cprint(f"[{side}/grasp] invalid AnyGrasp candidate {index}: {exc}", "yellow")

        scores = self._normalise([c["anygrasp_score"] for c in candidates])
        for candidate, score in zip(candidates, scores):
            candidate["anygrasp_normalized"] = float(score)
        return candidates

    def _grasp_target_config(
        self, candidate, side, obs_pose, high_pregrasp_offset_m=None
    ):
        import numpy as np
        from scipy.spatial.transform import Rotation as R

        scoring = self.config.get_grasp_scoring(side)
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
        low_pre[2, 3] += float(scoring.get("pregrasp_offset_m", 0.02))
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
                f"invalid execution_offset_base_m for {side}: {execution_offset}"
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
            "struct": self.config.get_arm_config(side).get("twin_struct", f"{side}_arm"),
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
        if self.config.sim_mode and candidate.get("suction_mode"):
            suction_cfg = self.config.shared.get("sim_suction", {})
            result["sim_suction"] = True
            result["xyz_threshold"] = float(
                suction_cfg.get("twin_xyz_threshold_m", 0.05)
            )
            result["rpy_threshold"] = float(
                suction_cfg.get("twin_rpy_threshold_rad", 0.03)
            )
        return result

    def _plan_grasp_candidate(self, candidate, side, obs_pose):
        """Use Twin once as a reachability test and cache its trajectory."""
        from termcolor import cprint
        scoring = self.config.get_grasp_scoring(side)
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
                cnfg = self._grasp_target_config(
                    candidate, side, obs_pose,
                    high_pregrasp_offset_m=high_offset,
                )
                rsp = self.twin_for(side).generate_trajectory2(cnfg)
                if (
                    not bool(rsp.get("value"))
                    and candidate.get("twin_generation3_fallback", True)
                ):
                    cprint(
                        f"[{side}/grasp] generation2 未通过，尝试 Twin robust generation3",
                        "yellow",
                    )
                    rsp = self.twin_for(side).call_service(
                        "trajectory_generation3", cnfg
                    )
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
                return True
            candidate["twin_reachable"] = 0.0
            candidate["twin_error"] = last_error
            return False
        except socket.timeout:
            candidate["twin_reachable"] = 0.0
            candidate["twin_error"] = "Twin request timeout"
            self._drop_twin(side)
            raise
        except Exception as exc:
            candidate["twin_reachable"] = 0.0
            candidate["twin_error"] = str(exc)
            self._drop_twin(side)
            cprint(f"[{side}/grasp] Twin reachability failed: {exc}", "yellow")
            return False

    def _score_grasp_candidates(self, candidates, side):
        """Combine AnyGrasp, Twin, width and approach-angle preferences."""
        import math
        import numpy as np

        cfg = self.config.get_grasp_scoring(side)
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
            candidate["pre_twin_score"] = float(
                float(weights.get("anygrasp", 0.45)) * candidate.get("anygrasp_normalized", 0.0)
                + float(weights.get("width", 0.15)) * width_score
                + float(weights.get("length", 0.05)) * length_score
                + float(weights.get("angle", 0.10)) * angle_score
            )
            candidate["composite_score"] = float(
                candidate["pre_twin_score"]
                + float(weights.get("twin", 0.30)) * candidate.get("twin_reachable", 0.0)
            )
        return sorted(candidates, key=lambda c: c["composite_score"], reverse=True)

    def _plan_best_grasp_candidate(self, candidates, side, obs_pose):
        """Plan only until the highest-scoring reachable candidate is found.

        Twin contributes the same fixed score to every reachable candidate,
        so sorting by the non-Twin score and stopping at the first reachable
        candidate preserves the previous winner without planning lower-ranked
        candidates that cannot win.
        """
        ranked = self._score_grasp_candidates(candidates, side)
        ranked.sort(key=lambda candidate: candidate["pre_twin_score"], reverse=True)
        planned_count = 0
        for candidate in ranked:
            planned_count += 1
            if self._plan_grasp_candidate(candidate, side, obs_pose):
                return (
                    candidate,
                    self._score_grasp_candidates(ranked, side),
                    planned_count,
                )
        return None, self._score_grasp_candidates(ranked, side), planned_count

    def _recover_grasp_failure(self, side, obs_pose):
        """Return the arm to a known observation pose and open the gripper."""
        from termcolor import cprint
        try:
            arm = self.arm_for(side)
            hand = self.gripper_for(side)
            scoring = self.config.get_grasp_scoring(side)
            if obs_pose:
                arm.move_to_named_pose(
                    obs_pose,
                    speed=int(scoring.get("grasp_recovery_speed", 10)),
                )
            hand.open()
        except Exception as exc:
            cprint(f"[{side}/grasp] recovery failed: {exc}", "red")

    def _execute_scored_grasp(self, candidate, side, obs_pose, hold_after_grasp=False):
        """Execute a cached Twin plan and verify contact before/after lifting.

        ``hold_after_grasp`` is used by handover state machines: after the
        gripper confirms contact, leave the arm at the grasp pose so a
        separately recorded handover trajectory can continue from there.
        The default keeps the historical behavior of returning to the
        observation pose and verifying the lift.
        """
        import time
        import numpy as np
        from termcolor import cprint
        scoring = self.config.get_grasp_scoring(side)
        arm = self.arm_for(side)
        hand = self.gripper_for(side)
        try:
            # Never send a grasp trajectory while the fingers may still be
            # closed around a previous object or may have stopped mid-motion.
            # Re-open and verify before every candidate, including runtime
            # retries after a missed grasp.
            if not hand.is_fully_open():
                cprint(f"[{side}/grasp] gripper not open; opening before grasp", "yellow")
                hand.open()
                time.sleep(0.3)
            if not hand.is_fully_open():
                raise RuntimeError("gripper is not fully open before grasp")
            if self.config.sim_mode and candidate.get("suction_mode"):
                set_target = getattr(hand, "set_suction_target", None)
                target = candidate.get("depth_anchor_left")
                if callable(set_target) and target is not None:
                    set_target(target)
            # Twin returns radians; arm services use degrees.
            trajectory = np.asarray(candidate["trajectory"], dtype=float) * 180.0 / np.pi
            if not arm.execute_trajectory(
                trajectory,
                speed=int(scoring.get("grasp_trajectory_speed", 12)),
            ):
                raise RuntimeError("arm trajectory execution failed")
            cprint(
                f"[{side}/grasp] geometry: cam_trans="
                f"{candidate.get('camera_translation_m')} "
                f"hand_base="
                f"{candidate.get('target_hand_position_base_m', candidate.get('hand_position_base_m'))} "
                f"end_target="
                f"{candidate.get('target_end_position_base_m')} "
                f"hand_to_end_t={candidate.get('hand_to_end_translation_m')}",
                "cyan",
            )
            close_resp = hand.close(
                force=int(scoring.get("gripper_close_force", 20)),
                speed=int(scoring.get("gripper_close_speed", 20)),
                soft=True,
            )
            time.sleep(0.3)
            response_info = close_resp.get("info", {}) if isinstance(close_resp, dict) else {}
            if not isinstance(response_info, dict):
                response_info = {}
            detected = close_resp.get("object_detected")
            if detected is None:
                # SimGripperClient and the real gripper service may wrap
                # their contact result under ``info``.
                detected = response_info.get("object_detected")
            if self.config.sim_mode and candidate.get("suction_mode"):
                distance = response_info.get("suction_distance_m")
                if distance is None:
                    distance = response_info.get("nearest_distance_m")
                cprint(
                    f"[{side}/grasp] sim contact: detected={bool(detected)} "
                    f"distance={distance if distance is not None else 'n/a'}m",
                    "green" if detected else "yellow",
                )
            if detected is None:
                detected = hand.is_grasping(
                    force=int(scoring.get("gripper_close_force", 20))
                )
            if not bool(detected):
                cprint(f"[{side}/grasp] no object detected after close", "yellow")
                self._recover_grasp_failure(side, obs_pose)
                return False

            # Soft close holds at force=0 (server-side design to protect
            # delicate objects); firm up before lifting or smooth/heavy
            # objects (metal cans) slip out during the lift motion.
            hand.close(
                force=int(scoring.get("gripper_hold_force", 80)),
                speed=int(scoring.get("gripper_close_speed", 20)),
            )
            time.sleep(0.3)

            if not hold_after_grasp and obs_pose and not arm.move_to_named_pose(
                obs_pose,
                speed=int(scoring.get("grasp_post_lift_speed", 15)),
            ):
                raise RuntimeError("post-grasp lift/return failed")
            if hold_after_grasp:
                cprint(
                    f"[{side}/grasp] object confirmed; holding at grasp pose",
                    "green",
                )
            elif (
                scoring.get("verify_lift", True)
                and not hand.is_grasping(
                    force=int(scoring.get("gripper_close_force", 20))
                )
            ):
                cprint(f"[{side}/grasp] object lost after lift", "yellow")
                self._recover_grasp_failure(side, obs_pose)
                return False
            self._set_runtime_attr("_last_successful_grasp_candidate", candidate)
            cprint(
                f"[{side}/grasp] selected candidate {candidate['index']} "
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
            cprint(f"[{side}/grasp] execution failed: {exc}", "red")
            self._recover_grasp_failure(side, obs_pose)
            return False

    def visual_grasp(
        self,
        object_name,
        side="left",
        location="desk_front",
        hold_after_grasp=False,
        observation_pose=None,
        use_vlm_grounding=True,
    ):
        """Unified left/right RGB-D grasp entry point used by all skills.

        When ``hold_after_grasp`` is true, a successful gripper contact leaves
        the arm at its grasp pose for a subsequent handover trajectory.
        """
        from termcolor import cprint

        started_total = time.perf_counter()
        timings = {}
        twin_candidates_planned = 0

        def record_timing(name, started):
            timings[name] = timings.get(name, 0.0) + time.perf_counter() - started

        def finish(success):
            timings["total_s"] = time.perf_counter() - started_total
            timings["twin_candidates_planned"] = twin_candidates_planned
            self._set_runtime_attr("_last_grasp_timings", dict(timings))
            phases = " ".join(
                f"{name[:-2]}={value:.3f}s"
                for name, value in timings.items()
                if name.endswith("_s")
            )
            cprint(
                f"[{side}/grasp] timing: {phases} "
                f"twin_candidates={twin_candidates_planned}",
                "cyan",
            )
            return success

        try:
            self.arm_for(side)
            self.gripper_for(side)
        except Exception as exc:
            cprint(f"[{side}/grasp] hardware connection failed: {exc}", "red")
            return finish(False)

        detector_prompts = None
        target_box = None
        # Do not expose a previous task's candidate if this invocation fails.
        self._set_runtime_attr("_last_successful_grasp_candidate", None)
        self._set_runtime_attr("_last_grasp_candidates", [])
        for pose_name, obs_pose in self._grasp_observation_poses(
            side, location, observation_pose=observation_pose
        ):
            try:
                if pose_name != "current":
                    scoring = self.config.get_grasp_scoring(side)
                    started = time.perf_counter()
                    if not self.control_arm(
                        pose_type=pose_name,
                        speed=int(scoring.get("grasp_observation_speed", 15)),
                        side=side,
                    ):
                        record_timing("observation_move_s", started)
                        continue
                    record_timing("observation_move_s", started)
                started = time.perf_counter()
                self.control_hand(cmd_type="open", side=side)
                record_timing("gripper_open_s", started)
                started = time.perf_counter()
                rgb, depth = self.get_camera_obs(side)
                record_timing("camera_capture_s", started)
                self.rgb, self.depth = rgb, depth
                started = time.perf_counter()
                self.save_current_transformation(side)
                record_timing("transform_update_s", started)
                camera = self.get_camera(side)
                camera_intrinsics = getattr(
                    camera, "intrinsics",
                    self.config.get_camera_intrinsics(side),
                )
                depth_scale = getattr(camera, "depth_scale", None)
                cprint(
                    f"[{side}/grasp] camera calibration: serial="
                    f"{getattr(camera, 'serial', 'n/a')} "
                    f"depth_scale_m={depth_scale if depth_scale is not None else 0.001} "
                    f"intrinsics={camera_intrinsics}",
                    "cyan",
                )
                started = time.perf_counter()
                raw = self.perception.detect_grasps(
                    rgb, depth, side=side, intrinsics=camera_intrinsics,
                    depth_scale=depth_scale,
                )
                record_timing("anygrasp_s", started)
                if not raw:
                    cprint(f"[{side}/grasp] no AnyGrasp candidates at {pose_name}", "yellow")
                    continue
                # The single-object path can use YOLOE directly.
                # In that mode the requested English class is passed to YOLO
                # and no VLM box is allowed to reject otherwise valid
                # AnyGrasp points.  VLM grounding remains opt-in for callers
                # that need phrase expansion or instance disambiguation.
                if detector_prompts is None:
                    if use_vlm_grounding:
                        started = time.perf_counter()
                        grounding = self.vlm.ground_object(rgb, object_name)
                        record_timing("grounding_s", started)
                        detector_prompts = grounding.get("prompts", [])
                        target_box = grounding.get("box")
                        if not detector_prompts:
                            detector_prompts = [object_name]
                    else:
                        detector_prompts = [object_name]
                        target_box = None
                cprint(
                    f"[{side}/grasp] VLM prompts: {detector_prompts}",
                    "cyan",
                )
                if target_box is not None:
                    cprint(f"[{side}/grasp] VLM target box: {target_box}", "cyan")
                started = time.perf_counter()
                filtered = self.perception.filter_grasps_by_detection(
                    raw, rgb, class_name=detector_prompts, side=side,
                    intrinsics=camera_intrinsics,
                    vis=bool(
                        self.config.shared.get("grasp_debug_visualization", False)
                    ),
                    target_box=target_box,
                )
                record_timing("detection_filter_s", started)
                started = time.perf_counter()
                candidates = self._build_grasp_candidates(filtered, side)
                record_timing("candidate_build_s", started)
                if not candidates:
                    continue
                started = time.perf_counter()
                candidate, ranked, planned_count = self._plan_best_grasp_candidate(
                    candidates, side, obs_pose
                )
                twin_candidates_planned += planned_count
                record_timing("twin_planning_s", started)
                self._set_runtime_attr("_last_grasp_candidates", ranked)
                if candidate is None:
                    cprint(f"[{side}/grasp] no Twin-reachable candidate at {pose_name}", "yellow")
                    continue
                started = time.perf_counter()
                succeeded = self._execute_scored_grasp(
                    candidate,
                    side,
                    obs_pose,
                    hold_after_grasp=hold_after_grasp,
                )
                record_timing("execution_s", started)
                if succeeded:
                    return finish(True)
                # A physical failure can move the object. Do not try a
                # second pose computed from this stale RGB-D frame.
                detector_prompts = None
                target_box = None
            except Exception as exc:
                cprint(f"[{side}/grasp] observation '{pose_name}' failed: {exc}", "red")
                self._recover_grasp_failure(side, obs_pose)
        cprint(f"[{side}/grasp] all observation poses failed", "red")
        return finish(False)

    def _save_grasp_visualization(
        self, image, grasp_points, valid_indices, valid_boxes, class_names,
    ):
        """Overwrite the most recent ``rgb_*.png`` with an annotated version.

        Annotations:
          - Red rectangle + class label around each YOLO detection box.
          - Small blue dot for every AnyGrasp candidate.
          - Green star for grasps that fall inside a detection box.
        """
        import glob
        import cv2

        rgb_files = sorted(
            glob.glob(os.path.join(self.save_path, "rgb_*.png")),
            key=os.path.getmtime,
        )
        if not rgb_files:
            return
        target_path = rgb_files[-1]

        img_bgr = cv2.cvtColor(image.copy(), cv2.COLOR_RGB2BGR)
        valid_indices = set(valid_indices or [])

        if len(grasp_points) > 0:
            for i, (u, v) in enumerate(grasp_points):
                if i in valid_indices:
                    continue
                cv2.circle(img_bgr, (int(u), int(v)), 3, (255, 0, 0), -1)
            for i in valid_indices:
                u, v = grasp_points[i]
                cv2.drawMarker(
                    img_bgr, (int(u), int(v)), (0, 255, 0),
                    markerType=cv2.MARKER_STAR, markerSize=14, thickness=2,
                )

        label_text = ",".join(class_names) if class_names else ""
        for box in valid_boxes:
            x1, y1, x2, y2 = (int(b) for b in box)
            cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 0, 255), 2)
            if label_text:
                cv2.putText(
                    img_bgr, label_text, (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA,
                )

        cv2.imwrite(target_path, img_bgr)
