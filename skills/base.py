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
        self._camera = None
        self._twin = None
        self._twins = {}  # per-side TwinClient cache: {"left": client, "right": client}
        self._perception = None
        self._vlm = None
        self._transforms = None
        self._json_parser = None
        self._tf_broadcaster = None

    # ------------------------------------------------------------------
    # Lazy property: arm client (socket to :8010)
    # ------------------------------------------------------------------
    @property
    def arm(self):
        """Return an initialized ArmController + connected socket client."""
        if self._arm is None:
            from core.arm import ArmClient
            if self.config._is_new_format:
                left_cfg = self.config.get_arm_config("left")
                host = self.config.shared.get("host", "127.0.0.1")
                port = left_cfg.get("arm_port", 8010)
            else:
                from core.config import HOST, ARM_PORT
                host, port = HOST, ARM_PORT
            client = ArmClient(host, port)
            client.connect()
            self._arm = client
        return self._arm

    # ------------------------------------------------------------------
    # Lazy property: hand client (socket to :8000)
    # ------------------------------------------------------------------
    @property
    def hand(self):
        """Return a connected HandClient."""
        if self._hand is None:
            from core.hand import HandClient
            if self.config._is_new_format:
                left_cfg = self.config.get_arm_config("left")
                host = self.config.shared.get("host", "127.0.0.1")
                port = left_cfg.get("hand_port", 8000)
            else:
                from core.config import HOST, HAND_PORT
                host, port = HOST, HAND_PORT
            client = HandClient(host, port)
            client.connect()
            self._hand = client
        return self._hand

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
            if self.config._is_new_format:
                host = self.config.shared.get("host", "127.0.0.1")
                port_key = "twin_port_left" if side == "left" else "twin_port_right"
                default_port = 8020 if side == "left" else 8021
                port = self.config.shared.get(port_key, default_port)
            else:
                from core.config import HOST, TWIN_PORT, TWIN_PORT_RIGHT
                host = HOST
                port = TWIN_PORT if side == "left" else TWIN_PORT_RIGHT
            client = TwinClient(host, port)
            client.connect()
            self._twins[side] = client
        return self._twins[side]

    # ------------------------------------------------------------------
    # Lazy property: camera
    # ------------------------------------------------------------------
    @property
    def camera(self):
        """Return an initialized RealSenseCapture instance (default: left arm camera)."""
        if self._camera is None:
            self._camera = self._make_camera("left")
        return self._camera

    def _make_camera(self, side="left"):
        """Create a RealSenseCapture bound to the camera of the given arm side."""
        from core.camera import RealSenseCapture
        serial = ""
        if self.config._is_new_format:
            arm_cfg = self.config.get_arm_config(side)
            serial = arm_cfg.get("camera_serial", "")
        self._camera_for_side = side
        return RealSenseCapture(
            width=640, height=480, fps=30, save_path=self.save_path, serial=serial,
        )

    def get_camera(self, side):
        """Return a RealSenseCapture for the requested arm side, caching per side."""
        if not hasattr(self, "_cameras"):
            self._cameras = {}
        if side not in self._cameras:
            self._cameras[side] = self._make_camera(side)
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
                anygrasp_host=host,
                anygrasp_port=port,
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
        import json, struct
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

    def control_hand(self, cmd_type="close"):
        """Control the dexterous hand (open / close / get_state)."""
        if cmd_type == "close":
            self.hand.close()
        elif cmd_type == "open":
            self.hand.open()
        elif cmd_type == "get_state":
            return self.hand.get_state()

    def check_grasping_object(self):
        """Detect whether the hand is holding an object."""
        return self.hand.is_grasping()

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

    def control_arm(self, pose_type=None, trajectory=None, speed=20):
        """Move the arm to a named pose or along a joint-space trajectory."""
        import numpy as np
        from core.transition import is_transition_allowed
        try:
            arm = self.arm
            if pose_type is not None:
                pose = self.config.get_pose(pose_type)
                if pose is None:
                    raise KeyError(f"Pose '{pose_type}' not found in config")
                adjacency = self.config.get_arm_config("left").get("transition_adjacency", "free")
                last_pose = getattr(self, "_last_named_pose", "home")
                if not is_transition_allowed(last_pose, pose_type, adjacency):
                    from termcolor import cprint
                    cprint(f"[transition] {last_pose} → {pose_type} not allowed, routing through home", "yellow")
                    home_pose = self.config.get_pose("home")
                    if home_pose:
                        arm.move_to_named_pose(home_pose, speed=speed)
                arm.move_to_named_pose(pose, speed=speed)
                self._last_named_pose = pose_type
            elif trajectory is not None:
                arm.execute_trajectory(trajectory, speed=speed)
            return True
        except Exception as e:
            from termcolor import cprint
            cprint(f"Arm control error: {e}", "red")
            return False

    def save_current_transformation(self):
        """Cache the base-to-camera and hand-effector-to-arm-endlink transforms."""
        from_frame = self.config.base_link_name
        to_frame = self.config.camera_link_name
        self.T_base_to_cam, _, _ = (
            self.transforms.get_transform_from_frame_to_frame(from_frame, to_frame)
        )
        grasping_from_frame = self.config.hand_effector_name
        grasping_to_frame = self.config.arm_end_link_name
        self.T_hand_effector_to_arm_endlink, _, _ = (
            self.transforms.get_transform_from_frame_to_frame(
                grasping_from_frame, grasping_to_frame
            )
        )

    def get_camera_obs(self, side="left"):
        """Capture a single RGB-D frame from the specified arm's camera."""
        cam = self.get_camera(side)
        rgb, depth = cam.get_rgbd()
        return rgb, depth

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


# ---------------------------------------------------------------------------
# Dual-Arm Skill Base
# ---------------------------------------------------------------------------
class DualArmSkill(Skill):
    """
    Base class for dual-arm skills.

    Holds two :class:`ArmSide` instances (``left`` / ``right``) plus a
    :class:`DualArmSync` for coordinated operations.  Shared resources
    (camera, twin, perception, vlm, transforms, etc.) are inherited from
    :class:`Skill`.
    """

    def __init__(self, config_path="./robot_config.json", save_path="./log"):
        super().__init__(config_path, save_path)
        from core.arm_side import ArmSide
        from core.sync import DualArmSync

        if not self.config._is_new_format:
            raise RuntimeError(
                "DualArmSkill requires the new dual-arm config format "
                "(top-level 'arms' key in robot_config.json). "
                f"Found keys: {list(self.config._raw.keys())}"
            )

        host = self.config.shared.get("host", "127.0.0.1")
        self.left = ArmSide("left", self.config.get_arm_config("left"), host=host)
        self.right = ArmSide("right", self.config.get_arm_config("right"), host=host)
        self.sync = DualArmSync()

    # ------------------------------------------------------------------
    # Convenience: move both arms in parallel
    # ------------------------------------------------------------------
    def move_both(self, left_pose_type=None, right_pose_type=None, speed=20):
        """Move both arms to their respective named poses simultaneously."""
        from threading import Thread

        def _move_arm(arm_side, pose_name, spd):
            pose = arm_side.get_pose(pose_name)
            arm_side.arm.move_to_named_pose(pose, speed=spd)

        t1 = Thread(target=_move_arm, args=(self.left, left_pose_type, speed))
        t2 = Thread(target=_move_arm, args=(self.right, right_pose_type, speed))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

    # ------------------------------------------------------------------
    # Convenience: inter-arm handover
    # ------------------------------------------------------------------
    def handover(self, from_side="left", to_side="right"):
        """
        Transfer an object from one arm to the other.

        Both arms move to the handover pose pair in parallel, the *giver*
        opens its hand, and the *receiver* closes.
        """
        from threading import Thread

        # Retrieve the handover pose pair from config
        key = f"{from_side}_to_{to_side}_handover"
        poses = self.config.get_dual_arm_pose(key)
        left_pose = poses.get("left_pose")
        right_pose = poses.get("right_pose")
        if left_pose is None or right_pose is None:
            raise KeyError(
                f"Dual-arm handover pose '{key}' must contain both "
                f"'left_pose' and 'right_pose', got: {list(poses.keys())}"
            )

        # Move both arms to their handover positions in parallel
        t1 = Thread(target=self.left.arm.move_to_named_pose, args=(left_pose,))
        t2 = Thread(target=self.right.arm.move_to_named_pose, args=(right_pose,))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        # Giver opens, receiver closes
        giver = self.left if from_side == "left" else self.right
        receiver = self.right if from_side == "left" else self.left
        giver.hand.open()
        receiver.hand.close()
