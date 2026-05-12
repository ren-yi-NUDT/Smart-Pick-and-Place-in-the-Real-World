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
        from core.backends import create_config
        self.config = create_config(config_path, save_path)
        self.config_path = config_path
        self.save_path = save_path
        os.makedirs(self.save_path, exist_ok=True)

        # Lazy-loaded hardware / service handles
        self._arm = None
        self._hand = None
        self._camera = None
        self._twin = None
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
            from core.backends import create_arm
            self._arm = create_arm()
            self._arm.connect()
        return self._arm

    # ------------------------------------------------------------------
    # Lazy property: hand client (socket to :8000)
    # ------------------------------------------------------------------
    @property
    def hand(self):
        """Return a connected HandClient."""
        if self._hand is None:
            from core.backends import create_hand
            self._hand = create_hand()
            self._hand.connect()
        return self._hand

    # ------------------------------------------------------------------
    # Lazy property: twin client (socket to :8020)
    # ------------------------------------------------------------------
    @property
    def twin(self):
        """Return a connected TwinClient."""
        if self._twin is None:
            from core.backends import create_twin
            self._twin = create_twin()
            self._twin.connect()
        return self._twin

    # ------------------------------------------------------------------
    # Lazy property: camera
    # ------------------------------------------------------------------
    @property
    def camera(self):
        """Return an initialized camera capture instance."""
        if self._camera is None:
            from core.backends import create_camera
            self._camera = create_camera(save_path=self.save_path)
        return self._camera

    # ------------------------------------------------------------------
    # Lazy property: perception (YOLO-World model)
    # ------------------------------------------------------------------
    @property
    def perception(self):
        """Return a loaded PerceptionModule (YOLO-World + AnyGrasp)."""
        if self._perception is None:
            from core.backends import create_perception
            self._perception = create_perception()
        return self._perception

    # ------------------------------------------------------------------
    # Lazy property: VLM client (GLM-4.5V API)
    # ------------------------------------------------------------------
    @property
    def vlm(self):
        """Return a VLMClient for GLM-4.5V vision-language calls."""
        if self._vlm is None:
            from core.backends import create_vlm
            self._vlm = create_vlm()
        return self._vlm

    # ------------------------------------------------------------------
    # Lazy property: transforms (ROS TF helper)
    # ------------------------------------------------------------------
    @property
    def transforms(self):
        """Return a TransformationUtil instance (ROS tf2)."""
        if self._transforms is None:
            from core.backends import create_transforms
            self._transforms = create_transforms()
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

    def control_arm(self, pose_type=None, trajectory=None, speed=20):
        """Move the arm to a named pose or along a joint-space trajectory."""
        import numpy as np
        try:
            arm = self.arm
            if pose_type is not None:
                pose = self.config.get_pose(pose_type)
                if pose is None:
                    raise KeyError(f"Pose '{pose_type}' not found in config")
                arm.move_to_named_pose(pose, speed=speed)
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

    def get_camera_obs(self):
        """Capture a single RGB-D frame."""
        rgb, depth = self.camera.get_rgbd()
        return rgb, depth

    # ------------------------------------------------------------------
    # Abstract run()
    # ------------------------------------------------------------------
    @abstractmethod
    def run(self, **kwargs):
        """Execute the skill.  Subclasses must override."""
        pass
