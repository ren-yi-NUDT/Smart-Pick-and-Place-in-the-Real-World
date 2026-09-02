#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Skill base class and registry for Smart Pick-and-Place.
"""

from abc import ABC, abstractmethod
import os
import socket

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

    Subclasses **must** implement ``run(**kwargs)``.
    Hardware clients are created lazily via properties so that importing a
    skill module never triggers a network / ROS connection on import.
    """

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
        self._tf_broadcaster = None
        # Runtime result of the most recent visual grasp. Composite skills
        # can use this fresh candidate for a subsequent placement trajectory.
        self._last_grasp_candidates = []
        self._last_successful_grasp_candidate = None

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
    # Lazy property: perception (YOLO-World model)
    # ------------------------------------------------------------------
    @property
    def perception(self):
        """Return a loaded PerceptionModule (YOLO-World + AnyGrasp)."""
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
    # Lazy property: TF broadcaster
    # ------------------------------------------------------------------
    @property
    def tf_broadcaster(self):
        """Return a ROS StaticTransformBroadcaster."""
        if self._tf_broadcaster is None:
            from tf2_ros import StaticTransformBroadcaster
            self._tf_broadcaster = StaticTransformBroadcaster()
        return self._tf_broadcaster

    # ------------------------------------------------------------------
    # Convenience helpers used across many skills
    # ------------------------------------------------------------------
    def send_cmd(self, sock_or_client, data):
        """Send a JSON command. Accepts client object or raw socket."""
        if hasattr(sock_or_client, '_send_cmd'):
            return sock_or_client._send_cmd(data)
        import json
        if hasattr(sock_or_client, 'sock') and sock_or_client.sock is not None:
            return sock_or_client._send_cmd(data)
        msg = json.dumps(data).encode("utf-8")
        sock_or_client.sendall(msg)
        return json.loads(sock_or_client.recv(1024).decode("utf-8"))

    def send_cmd_twin(self, twin_client_or_sock, data):
        """Send a command to the Twin service."""
        if hasattr(twin_client_or_sock, '_send_cmd'):
            return twin_client_or_sock._send_cmd(data)
        # Fallback: raw socket with length-prefix receive
        import json
        import struct
        msg = json.dumps(data).encode("utf-8")
        twin_client_or_sock.sendall(msg)
        length_bytes = b""
        while len(length_bytes) < 4:
            chunk = twin_client_or_sock.recv(4 - len(length_bytes))
            if not chunk:
                raise ConnectionError("Connection closed")
            length_bytes += chunk
        data_length = struct.unpack(">I", length_bytes)[0]
        data_bytes = b""
        while len(data_bytes) < data_length:
            chunk = twin_client_or_sock.recv(min(4096, data_length - len(data_bytes)))
            if not chunk:
                raise ConnectionError("Connection closed")
            data_bytes += chunk
        return json.loads(data_bytes.decode("utf-8"))

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

    def _left_arm_j2_pretension(self, speed=15):
        """Nudge left arm J2 by 10° toward 0 before a long return-home motion.

        When the arm has just finished a high-Z grasp, sending the home command
        directly lets the trajectory planner produce a path that dips downward
        mid-way and knocks over the workspace. A small J2 motion toward 0 first
        lifts the shoulder configuration and breaks up the bad interpolation.

        Reads current joints from ROS ``/joint_states``. **Filters by
        ``msg.name``** because the topic has multiple publishers (left arm
        publishes ``["joint1".."joint7"]`` at 20 Hz; the Inspire hand publishes
        its own hand joints at 15 Hz). Without filtering, ``wait_for_message``
        races and ~40% of calls receive the hand message — ``msg.position[:7]``
        then parses hand-joint radians as left-arm degrees, producing a garbage
        ``intermediate`` pose that drives the arm to a random configuration.
        Silently skips if a left-arm message doesn't arrive within 5 attempts.
        """
        from termcolor import cprint
        try:
            import rospy
            from sensor_msgs.msg import JointState
            LEFT_ARM_NAMES = {"joint1", "joint2", "joint3", "joint4",
                              "joint5", "joint6", "joint7"}
            msg = None
            for _ in range(5):
                cand = rospy.wait_for_message("/joint_states", JointState, timeout=2.0)
                if cand is not None and set(cand.name[:7]) == LEFT_ARM_NAMES:
                    msg = cand
                    break
            if msg is None:
                cprint("[J2_pretension] no left-arm /joint_states msg, skipped", "yellow")
                return
            current_deg = [r * 180.0 / 3.141592653589793 for r in msg.position[:7]]
            current_J2 = current_deg[1]
            if abs(current_J2) < 0.5:
                return
            new_J2 = current_J2 + (10.0 if current_J2 < 0 else -10.0)
            cprint(f"[J2_pretension] J2: {current_J2:.2f}° → {new_J2:.2f}°", "cyan")
            intermediate = [current_deg[0], new_J2] + current_deg[2:7]
            self.arm.execute_trajectory([intermediate], speed=speed)
        except Exception as e:
            cprint(f"[J2_pretension] skipped ({e})", "yellow")

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
                    and candidate.get("twin_generation3_fallback", False)
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
            candidate["composite_score"] = float(
                float(weights.get("anygrasp", 0.45)) * candidate.get("anygrasp_normalized", 0.0)
                + float(weights.get("twin", 0.30)) * candidate.get("twin_reachable", 0.0)
                + float(weights.get("width", 0.15)) * width_score
                + float(weights.get("length", 0.05)) * length_score
                + float(weights.get("angle", 0.10)) * angle_score
            )
        return sorted(candidates, key=lambda c: c["composite_score"], reverse=True)

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
            self._last_successful_grasp_candidate = candidate
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
        try:
            self.arm_for(side)
            self.gripper_for(side)
        except Exception as exc:
            cprint(f"[{side}/grasp] hardware connection failed: {exc}", "red")
            return False

        detector_prompts = None
        target_box = None
        # Do not expose a previous task's candidate if this invocation fails.
        self._last_successful_grasp_candidate = None
        self._last_grasp_candidates = []
        for pose_name, obs_pose in self._grasp_observation_poses(
            side, location, observation_pose=observation_pose
        ):
            try:
                if pose_name != "current":
                    scoring = self.config.get_grasp_scoring(side)
                    if not self.control_arm(
                        pose_type=pose_name,
                        speed=int(scoring.get("grasp_observation_speed", 15)),
                        side=side,
                    ):
                        continue
                self.control_hand(cmd_type="open", side=side)
                rgb, depth = self.get_camera_obs(side)
                self.rgb, self.depth = rgb, depth
                self.save_current_transformation(side)
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
                raw = self.perception.detect_grasps(
                    rgb, depth, side=side, intrinsics=camera_intrinsics,
                    depth_scale=depth_scale,
                )
                if not raw:
                    cprint(f"[{side}/grasp] no AnyGrasp candidates at {pose_name}", "yellow")
                    continue
                # The legacy single-object path can use YOLO-World directly.
                # In that mode the requested English class is passed to YOLO
                # and no VLM box is allowed to reject otherwise valid
                # AnyGrasp points.  VLM grounding remains opt-in for callers
                # that need phrase expansion or instance disambiguation.
                if detector_prompts is None:
                    if use_vlm_grounding:
                        grounding = self.vlm.ground_object(rgb, object_name)
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
                filtered = self.perception.filter_grasps_by_detection(
                    raw, rgb, class_name=detector_prompts, side=side,
                    intrinsics=camera_intrinsics, vis=True,
                    target_box=target_box,
                )
                candidates = self._build_grasp_candidates(filtered, side)
                if not candidates:
                    continue
                for candidate in candidates:
                    self._plan_grasp_candidate(candidate, side, obs_pose)
                if not any(c.get("twin_reachable") for c in candidates):
                    cprint(f"[{side}/grasp] no Twin-reachable candidate at {pose_name}", "yellow")
                    continue
                # Rank all candidates, including unreachable ones, so the
                # Twin term is a real part of the composite score rather than
                # a post-filter that is always equal to one.
                ranked = self._score_grasp_candidates(candidates, side)
                self._last_grasp_candidates = ranked
                for candidate in ranked:
                    if not candidate.get("twin_reachable"):
                        continue
                    if self._execute_scored_grasp(
                        candidate,
                        side,
                        obs_pose,
                        hold_after_grasp=hold_after_grasp,
                    ):
                        return True
                    # A physical failure can move the object. Do not try a
                    # second pose computed from this stale RGB-D frame.
                    detector_prompts = None
                    target_box = None
                    break
            except Exception as exc:
                cprint(f"[{side}/grasp] observation '{pose_name}' failed: {exc}", "red")
                self._recover_grasp_failure(side, obs_pose)
        cprint(f"[{side}/grasp] all observation poses failed", "red")
        return False

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

    # ------------------------------------------------------------------
    # Abstract run()
    # ------------------------------------------------------------------
    @abstractmethod
    def run(self, **kwargs):
        """Execute the skill.  Subclasses must override."""
        pass
