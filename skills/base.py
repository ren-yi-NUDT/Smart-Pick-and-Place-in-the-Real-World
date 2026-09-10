#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Skill base class and registry for Smart Pick-and-Place.
"""

from abc import ABC, abstractmethod
import os

# ---------------------------------------------------------------------------
# Skill Registry
# ---------------------------------------------------------------------------
_SKILL_REGISTRY = {}


def register_skill(name):
    """Decorator to register a skill class under a given name."""
    def decorator(cls):
        _SKILL_REGISTRY[name] = cls
        return cls
    return decorator


def get_skill(name):
    """Return the skill class registered under *name*, or None."""
    return _SKILL_REGISTRY.get(name)


def list_skills():
    """Return a sorted list of all registered skill names."""
    return sorted(_SKILL_REGISTRY.keys())


# ---------------------------------------------------------------------------
# Base Skill
# ---------------------------------------------------------------------------
class Skill(ABC):
    """
    Abstract base for every robot skill.

    Subclasses implement ``execute(**kwargs)``.  ``run`` is the common
    lifecycle entry point.  A temporary ``__init_subclass__`` adapter keeps
    older third-party/local Skills that still implement ``run`` working while
    they are migrated.
    Hardware clients are created lazily via properties so that importing a
    skill module never triggers a network / ROS connection on import.
    """

    def __init_subclass__(cls, **kwargs):
        """Adapt legacy subclasses that override ``run`` directly.

        This makes the lifecycle uniform at the framework boundary without
        forcing every downstream Skill to change in the same deployment.
        """
        super().__init_subclass__(**kwargs)
        legacy_run = cls.__dict__.get("run")
        explicit_execute = cls.__dict__.get("execute")
        if legacy_run is not None and explicit_execute is None:
            setattr(cls, "execute", legacy_run)
            setattr(cls, "run", Skill.run)

    def __init__(self, config_path="./robot_config.json", save_path="./log"):
        from core.config import Config
        self.config = Config(config_path)
        self.config_path = config_path
        self.save_path = save_path
        os.makedirs(self.save_path, exist_ok=True)

        # Lazy-loaded hardware / service handles
        self._arm = None
        self._hand = None
        self._arms = {}
        self._hands = {}
        self._camera = None
        self._cameras = {}
        self._twin = None
        self._twins = {}  # per-side TwinClient cache: {"left": client, "right": client}
        self._perception = None
        self._vlm = None
        self._transforms = None
        self._json_parser = None
        # Runtime result of the most recent visual grasp. Composite skills
        # can use this fresh candidate for a subsequent placement trajectory.
        self._last_grasp_candidates = []
        self._last_successful_grasp_candidate = None
        self._last_grasp_timings = {}
        from core.skill_runtime import RobotContext
        self.context = RobotContext(self)
        self.last_result = None
        self.lifecycle_state = "idle"

    def validate_inputs(self, **kwargs):
        """Optional precondition hook for all Skills."""
        return None

    def recover(self, result):
        """Optional failure-recovery hook.

        It is deliberately a no-op by default: blindly moving a physical
        robot during generic exception handling is less safe than stopping.
        Pipelines should implement explicit, state-aware recovery instead.
        """
        return None

    def cleanup(self):
        """Close clients owned by this Skill after every run.

        A Skill may be invoked repeatedly by a long-running process.  Leaving
        cached TCP clients alive after a run can strand an idle connection in
        a bridge that still has a legacy single-client handler.  Gripper
        clients use ``close_connection`` because ``close`` is a physical
        gripper command, not a socket lifecycle method.
        """
        clients = []
        clients.extend(self._arms.values())
        clients.extend(self._hands.values())
        clients.extend(self._twins.values())
        clients.extend(self._cameras.values())
        if self._camera is not None:
            clients.append(self._camera)
        if self._perception is not None:
            anygrasp_client = getattr(self._perception, "_anygrasp_client", None)
            if anygrasp_client is not None:
                clients.append(anygrasp_client)

        seen = set()
        for client in clients:
            if client is None or id(client) in seen:
                continue
            seen.add(id(client))
            closer = getattr(client, "close_connection", None)
            if not callable(closer):
                closer = getattr(client, "close", None)
            if callable(closer):
                try:
                    closer()
                except Exception:
                    pass

        self._arms.clear()
        self._hands.clear()
        self._twins.clear()
        self._cameras.clear()
        self._arm = None
        self._hand = None
        self._camera = None
        self._twin = None
        self._right_arm = None
        self._right_gripper = None
        if self._perception is not None:
            self._perception._anygrasp_client = None

    # ------------------------------------------------------------------
    # Lazy property: arm client (socket to :8010)
    # ------------------------------------------------------------------
    @property
    def arm(self):
        """Backward-compatible alias for :meth:`arm_for('left')`."""
        if self._arm is None:
            self._arm = self.arm_for("left")
        return self._arm

    def arm_for(self, side="left"):
        """Return the connected arm client for the requested side."""
        if side not in ("left", "right"):
            raise ValueError(f"Unsupported arm side: {side}")
        if side in self._arms:
            return self._arms[side]
        arm_cfg = self.config.get_arm_config(side)
        host = self.config.shared.get("host", "127.0.0.1")
        if self.config.sim_mode:
            from core.sim_arm import SimArmClient
            client = SimArmClient(host, 8031, side=side)
        else:
            from core.arm import ArmClient
            client = ArmClient(
                host, arm_cfg.get("arm_port", 8010), side=side
            )
        if not client.connect():
            raise ConnectionError(f"Unable to connect to {side} arm")
        self._arms[side] = client
        if side == "left":
            self._arm = client
        else:
            self._right_arm = client
        return client

    # ------------------------------------------------------------------
    # Lazy property: hand client (Robotiq 85 gripper)
    # ------------------------------------------------------------------
    @property
    def hand(self):
        """Backward-compatible alias for :meth:`gripper_for('left')`."""
        if self._hand is None:
            self._hand = self.gripper_for("left")
        return self._hand

    def gripper_for(self, side="left"):
        """Return the connected gripper for *side*; never silently mock real hardware."""
        if side not in ("left", "right"):
            raise ValueError(f"Unsupported arm side: {side}")
        if side in self._hands:
            return self._hands[side]
        arm_cfg = self.config.get_arm_config(side)
        host = self.config.shared.get("host", "127.0.0.1")
        src = f"/{side}_gripper/movement_control"
        if self.config.sim_mode:
            from core.sim_gripper import SimGripperClient
            client = SimGripperClient(host, 8031, src=src)
        else:
            from core.gripper import GripperClient
            client = GripperClient(
                host, arm_cfg.get("hand_port", 8002), src=src, allow_mock=False
            )
        if not client.connect():
            raise ConnectionError(f"Unable to connect to {side} gripper")
        self._hands[side] = client
        if side == "left":
            self._hand = client
        else:
            self._right_gripper = client
        return client

    # ------------------------------------------------------------------
    # Lazy property: twin client (socket to :8020 for left arm)
    # ------------------------------------------------------------------
    @property
    def twin(self):
        """Return a connected TwinClient for the LEFT arm (port 8020, legacy default)."""
        if self._twin is None:
            self._twin = self.twin_for("left")
        return self._twin

    def twin_for(self, side="left"):
        """Return a connected TwinClient for the requested arm side.

        Left arm routes to port 8020, right arm to port 8021. Clients are
        cached per side so repeated calls reuse the same socket.
        """
        if side not in self._twins:
            from core.twin_client import TwinClient
            from core.config import SIM_TWIN_PORT_LEFT, SIM_TWIN_PORT_RIGHT
            host = self.config.shared.get("host", "127.0.0.1")
            if self.config.sim_mode:
                port_key = "sim_twin_port_left" if side == "left" else "sim_twin_port_right"
                default_port = SIM_TWIN_PORT_LEFT if side == "left" else SIM_TWIN_PORT_RIGHT
            else:
                port_key = "twin_port_left" if side == "left" else "twin_port_right"
                default_port = 8020 if side == "left" else 8021
            port = self.config.shared.get(port_key, default_port)
            client = TwinClient(host, port)
            if not client.connect():
                raise ConnectionError(f"Unable to connect to {side} Twin service")
            self._twins[side] = client
        return self._twins[side]

    def _drop_twin(self, side):
        """Close and forget a broken Twin connection so the next retry reconnects."""
        client = self._twins.pop(side, None)
        if client is not None:
            try:
                client.close()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Lazy property: camera
    # ------------------------------------------------------------------
    @property
    def camera(self):
        """Return an initialized RealSenseCapture instance (default: left arm camera)."""
        if self._camera is None:
            # Keep the legacy singular alias and the per-side cache pointing
            # at the same object. Creating these independently can leave a
            # RealSense pipeline alive after the other cache is closed.
            self._camera = self.get_camera("left")
        return self._camera

    def _make_camera(self, side="left"):
        """Create a camera bound to the given arm side (sim or real)."""
        self._camera_for_side = side
        if self.config.sim_mode:
            from core.sim_camera import SimCamera
            host = self.config.shared.get("host", "127.0.0.1")
            cam = SimCamera(width=640, height=480, fps=30, save_path=self.save_path,
                            serial="", host=host, port=8031, side=side)
            if not cam.connect():
                raise ConnectionError(f"Unable to connect to {side} simulation camera")
            return cam
        from core.camera import RealSenseCapture
        serial = self.config.get_arm_config(side).get("camera_serial", "")
        return RealSenseCapture(
            width=640, height=480, fps=30, save_path=self.save_path, serial=serial,
            intrinsics=self.config.get_camera_intrinsics(side),
        )

    def get_camera(self, side):
        """Return a RealSenseCapture for the requested arm side, caching per side."""
        if side not in ("left", "right"):
            raise ValueError(f"Unsupported camera side: {side}")
        if side not in self._cameras:
            self._cameras[side] = self._make_camera(side)
        if side == "left":
            self._camera = self._cameras[side]
        return self._cameras[side]

    # ------------------------------------------------------------------
    # Lazy property: perception (YOLOE-26 model)
    # ------------------------------------------------------------------
    @property
    def perception(self):
        """Return a loaded PerceptionModule (YOLOE-26 + AnyGrasp)."""
        if self._perception is None:
            from core.perception import Perception
            from core.config import DEFAULT_YOLO_MODEL, DEFAULT_ANYGRASP_CHECKPOINT
            host = self.config.shared.get("anygrasp_host", "127.0.0.1")
            port = self.config.shared.get("anygrasp_port", 8030)
            self._perception = Perception(
                yolo_model_path=DEFAULT_YOLO_MODEL,
                anygrasp_checkpoint=DEFAULT_ANYGRASP_CHECKPOINT,
                save_path=self.save_path,
                anygrasp_host=host,
                anygrasp_port=port,
                camera_intrinsics={
                    side: self.config.get_camera_intrinsics(side)
                    for side in ("left", "right")
                },
            )
        return self._perception

    # ------------------------------------------------------------------
    # Lazy property: VLM client (GLM-4.5V API)
    # ------------------------------------------------------------------
    @property
    def vlm(self):
        """Return a VLMClient for GLM-4.5V vision-language calls."""
        if self._vlm is None:
            from core.vlm import VLMClient
            self._vlm = VLMClient()
        return self._vlm

    # ------------------------------------------------------------------
    # Lazy property: transforms (ROS TF helper)
    # ------------------------------------------------------------------
    @property
    def transforms(self):
        """Return a TransformationUtil instance (ROS tf2)."""
        if self._transforms is None:
            from core.transforms import TransformationUtil
            self._transforms = TransformationUtil()
        return self._transforms

    # ------------------------------------------------------------------
    # Lazy property: JSON input parser
    # ------------------------------------------------------------------
    @property
    def json_parser(self):
        """Return a JsonInputParser instance."""
        if self._json_parser is None:
            from core.json_input import JsonInputParser
            self._json_parser = JsonInputParser()
        return self._json_parser

    # ------------------------------------------------------------------
    # Convenience helpers used across many skills
    # ------------------------------------------------------------------
    def send_cmd_twin(self, twin_client_or_sock, data):
        """Send a command to the Twin service."""
        if hasattr(twin_client_or_sock, '_send_cmd'):
            return twin_client_or_sock._send_cmd(data)
        from core.tcp_protocol import recv_json_compat, send_json_frame
        send_json_frame(twin_client_or_sock, data)
        return recv_json_compat(twin_client_or_sock)

    def control_hand(self, cmd_type="close", side="left", **kwargs):
        """Control one arm's gripper and return the service response."""
        hand = self.gripper_for(side)
        if cmd_type == "close":
            return hand.close(**kwargs)
        elif cmd_type == "open":
            return hand.open(**kwargs)
        elif cmd_type == "get_state":
            return hand.get_state()
        raise ValueError(f"Unknown gripper command: {cmd_type}")

    def check_grasping_object(self, side="left"):
        """Detect whether the hand is holding an object."""
        scoring = self.config.get_grasp_scoring(side)
        return self.gripper_for(side).is_grasping(
            force=int(scoring.get("gripper_close_force", 20))
        )

    def control_arm(self, pose_type=None, trajectory=None, speed=20, side="left"):
        """Move the arm to a named pose or along a joint-space trajectory."""
        from core.transition import is_transition_allowed
        try:
            arm = self.arm_for(side)
            if pose_type is not None:
                pose = self.config.get_pose(pose_type, side=side)
                if pose is None:
                    raise KeyError(f"Pose '{pose_type}' not found in config")
                adjacency = self.config.get_arm_config(side).get("transition_adjacency", "free")
                last_poses = getattr(self, "_last_named_poses", {})
                last_pose = last_poses.get(side, "home")
                if not is_transition_allowed(last_pose, pose_type, adjacency):
                    from termcolor import cprint
                    cprint(f"[{side}/transition] {last_pose} → {pose_type} not allowed, routing through home", "yellow")
                    home_pose = self.config.get_pose("home", side=side)
                    if home_pose:
                        if not arm.move_to_named_pose(home_pose, speed=speed):
                            return False
                ok = arm.move_to_named_pose(pose, speed=speed)
                if not ok:
                    return False
                last_poses[side] = pose_type
                self._last_named_poses = last_poses
            elif trajectory is not None:
                if not arm.execute_trajectory(trajectory, speed=speed):
                    return False
            return True
        except Exception as e:
            from termcolor import cprint
            cprint(f"Arm control error: {e}", "red")
            return False

    def save_current_transformation(self, side="left"):
        """Cache the base-to-camera and hand-effector-to-arm-endlink transforms."""
        arm_cfg = self.config.get_arm_config(side)
        if self.config.sim_mode:
            self._save_transforms_from_sim(side)
            return
        from_frame = arm_cfg["base_link_name"]
        to_frame = arm_cfg.get("camera_extrinsic", {}).get(
            "child_frame", self.config.camera_link_name
        )
        T_base_to_cam, _, _ = (
            self.transforms.get_transform_from_frame_to_frame(from_frame, to_frame)
        )
        grasping_from_frame = arm_cfg["hand_effector_name"]
        grasping_to_frame = arm_cfg["arm_end_link_name"]
        T_hand_effector_to_arm_endlink, _, _ = (
            self.transforms.get_transform_from_frame_to_frame(
                grasping_from_frame, grasping_to_frame
            )
        )
        # Preserve the project's established AnyGrasp/Twin convention for
        # this transform.  The real-hardware grasp candidates are expressed
        # in the same end/hand convention used by the original controller;
        # inverting it here changes otherwise reachable candidates to a
        # different wrist offset.  The simulated path has its own explicit
        # conversion in _save_transforms_from_sim().
        self._set_side_transforms(
            side, T_base_to_cam, T_hand_effector_to_arm_endlink
        )

    def _set_side_transforms(self, side, T_base_to_cam, T_hand_effector_to_arm_endlink):
        if not hasattr(self, "_side_transforms"):
            self._side_transforms = {}
        self._side_transforms[side] = (T_base_to_cam, T_hand_effector_to_arm_endlink)
        # Legacy consumers (placement and old helper methods) use these attrs.
        self.T_base_to_cam = T_base_to_cam
        self.T_hand_effector_to_arm_endlink = T_hand_effector_to_arm_endlink

    def _get_side_transforms(self, side):
        try:
            return self._side_transforms[side]
        except (AttributeError, KeyError):
            raise RuntimeError(f"Transforms for {side} arm have not been saved")

    def _select_best_container_grasp(self, rgb, depth, box, side="left"):
        """Select the highest-scoring AnyGrasp pose whose point is in *box*.

        The selected rotation is converted from the camera grasp convention
        to the arm-base hand convention, matching the normal grasp pipeline.
        The container detector still supplies the placement XYZ separately.
        """
        import numpy as np
        from core.transforms import graspcam2pixel, self_rotation_np

        try:
            camera = self.get_camera(side)
            intrinsics = getattr(
                camera, "intrinsics", self.config.get_camera_intrinsics(side)
            )
            raw_grasps = self.perception.detect_grasps(
                rgb, depth, side=side, intrinsics=intrinsics,
                depth_scale=getattr(camera, "depth_scale", None),
            )
            if not raw_grasps:
                return None
            points, _ = graspcam2pixel(
                raw_grasps, cam_type=side, intrinsics=intrinsics
            )
            x1, y1, x2, y2 = [float(value) for value in box]
            inside = [
                index for index, point in enumerate(points)
                if x1 < point[0] < x2 and y1 < point[1] < y2
            ]
            if not inside:
                return None

            best_index = max(
                inside,
                key=lambda index: float(raw_grasps[index].get("score", 0.0)),
            )
            grasp = raw_grasps[best_index]
            T_grasp = np.eye(4)
            T_grasp[:3, :3] = np.asarray(
                grasp["rotation_matrix"], dtype=float
            ).reshape(3, 3)
            T_grasp[:3, 3] = np.asarray(grasp["trans"], dtype=float).reshape(3)

            hand_convention = self_rotation_np(np.array([
                [0, 1, 0, 0], [-1, 0, 0, 0],
                [0, 0, 1, 0], [0, 0, 0, 1],
            ], dtype=float))
            if side == "left":
                hand_convention = hand_convention @ np.diag(
                    [-1.0, -1.0, 1.0, 1.0]
                )
            T_base_to_cam, _ = self._get_side_transforms(side)
            T_world_hand = T_base_to_cam @ (T_grasp @ hand_convention)
            if T_world_hand[:3, 0][0] < 0:
                T_world_hand = T_world_hand @ np.diag(
                    [-1.0, -1.0, 1.0, 1.0]
                )
            return {
                "rotation": T_world_hand[:3, :3].copy(),
                "score": float(grasp.get("score", 0.0)),
                "index": int(best_index),
            }
        except (KeyError, TypeError, ValueError, IndexError):
            return None

    def _save_transforms_from_sim(self, side="left"):
        """Sim-mode TF bypass: read link poses and express them in arm base.

        SimServer returns world-frame poses. This distinction matters for the
        right arm because its simulated base is translated and rotated 180°
        relative to the left arm.
        """
        import numpy as np
        from scipy.spatial.transform import Rotation as R

        def _pose_matrix(pos, orn, label):
            if not isinstance(pos, (list, tuple)) or not isinstance(orn, (list, tuple)):
                raise RuntimeError(f"SimServer returned incomplete {label} pose")
            if len(pos) != 3 or len(orn) != 4:
                raise RuntimeError(f"SimServer returned invalid {label} pose")
            T = np.eye(4)
            T[:3, :3] = R.from_quat(orn).as_matrix()
            T[:3, 3] = np.asarray(pos, dtype=float)
            return T

        def _link_pose(name):
            rsp = self.arm_for(side)._send({"cmd": "get_link_pose", "side": side, "link": name})
            info = rsp.get("info", {})
            return _pose_matrix(info.get("pos"), info.get("orn"), name)

        arm_cfg = self.config.get_arm_config(side)
        cam_name = arm_cfg.get("camera_extrinsic", {}).get(
            "child_frame", self.config.camera_link_name
        )
        base_rsp = self.arm_for(side)._send({"cmd": "get_base_pose", "side": side})
        base_info = base_rsp.get("info", {})
        T_world_to_base = np.linalg.inv(
            _pose_matrix(base_info.get("pos"), base_info.get("orn"), "base")
        )
        T_base_to_cam = T_world_to_base @ _link_pose(cam_name)
        T_base_to_end = T_world_to_base @ _link_pose(arm_cfg["arm_end_link_name"])
        T_base_to_hand = T_world_to_base @ _link_pose(arm_cfg["hand_effector_name"])
        self._set_side_transforms(
            side,
            T_base_to_cam,
            # Candidates are expressed at the hand-effector frame and Twin
            # expects the hand -> arm-end transform.  The previous order was
            # end -> hand, offsetting every simulated grasp away from the
            # visual gripper by the hand/flange distance.
            np.linalg.inv(T_base_to_hand) @ T_base_to_end,
        )

    def get_camera_obs(self, side="left"):
        """Capture a single RGB-D frame from the specified arm's camera."""
        cam = self.get_camera(side)
        rgb, depth = cam.get_rgbd()
        return rgb, depth

    # ------------------------------------------------------------------
    # Visual grasp pipeline (implementation in core/grasp_pipeline.py)
    # ------------------------------------------------------------------
    @property
    def grasp_pipeline(self):
        if getattr(self, "_grasp_pipeline_ref", None) is None:
            from core.grasp_pipeline import GraspPipeline

            self._grasp_pipeline_ref = GraspPipeline(
                getattr(self, "context", self)
            )
        return self._grasp_pipeline_ref

    def visual_grasp(self, object_name, **kwargs):
        """RGB-D → AnyGrasp → scoring → Twin plan → execute, for either arm."""
        return self.grasp_pipeline.visual_grasp(object_name, **kwargs)

    # ------------------------------------------------------------------
    # Placement pipeline (implementation in core/place_pipeline.py)
    # ------------------------------------------------------------------
    @property
    def place_pipeline(self):
        if getattr(self, "_place_pipeline_ref", None) is None:
            from core.place_pipeline import PlacePipeline

            self._place_pipeline_ref = PlacePipeline(
                getattr(self, "context", self)
            )
        return self._place_pipeline_ref

    @property
    def handover_pipeline(self):
        """Shared single-arm and dual-arm handover pipeline."""
        if getattr(self, "_handover_pipeline_ref", None) is None:
            from core.handover_pipeline import HandoverPipeline
            self._handover_pipeline_ref = HandoverPipeline(
                getattr(self, "context", self)
            )
        return self._handover_pipeline_ref

    @property
    def drawer_pipeline(self):
        """Shared drawer open/place/close pipeline."""
        if getattr(self, "_drawer_pipeline_ref", None) is None:
            from core.drawer_pipeline import DrawerPipeline
            self._drawer_pipeline_ref = DrawerPipeline(
                getattr(self, "context", self)
            )
        return self._drawer_pipeline_ref

    @property
    def observation_pipeline(self):
        """Shared workspace, drawer and handover observation pipeline."""
        if getattr(self, "_observation_pipeline_ref", None) is None:
            from core.observation_pipeline import ObservationPipeline
            self._observation_pipeline_ref = ObservationPipeline(
                getattr(self, "context", self)
            )
        return self._observation_pipeline_ref

    # ------------------------------------------------------------------
    # Common Skill lifecycle
    # ------------------------------------------------------------------
    def run(self, **kwargs):
        """Execute a Skill through the common lifecycle.

        The return type is now always :class:`SkillResult`, which remains
        boolean-compatible for existing composite code.
        """
        from core.skill_runtime import SkillResult, new_run_id, normalize_result
        from termcolor import cprint

        run_id = new_run_id()
        self.lifecycle_state = "preparing"
        try:
            self.validate_inputs(**kwargs)
            self.lifecycle_state = "executing"
            raw_result = self.execute(**kwargs)
            result = normalize_result(raw_result, run_id=run_id)
        except Exception as exc:
            result = SkillResult(
                ok=False,
                code="EXCEPTION",
                message="%s: %s" % (type(exc).__name__, exc),
                recoverable=False,
                run_id=run_id,
            )
            cprint("[skill] %s" % result.message, "red")
        if not result.ok:
            self.lifecycle_state = "recovering"
            try:
                self.recover(result)
            except Exception as exc:
                cprint("[skill] recovery hook failed: %s" % exc, "yellow")
        try:
            self.cleanup()
        except Exception as exc:
            cprint("[skill] cleanup failed: %s" % exc, "yellow")
        self.lifecycle_state = "finished"

        self.last_result = result
        return result

    @abstractmethod
    def execute(self, **kwargs):
        """Implement the task-specific operation."""
        raise NotImplementedError
