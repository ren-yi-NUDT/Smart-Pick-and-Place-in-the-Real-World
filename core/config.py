"""
Configuration loader and constants for Smart Pick-and-Place.

Usage:
    from core.config import Config
    cfg = Config()                       # loads robot_config.json from project root
    cfg = Config(config_path="/my/path")  # explicit path
"""

import json
import os

from core.abc import BaseConfig

# ---------------------------------------------------------------------------
# Project root (the directory that contains robot_config.json)
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# Network constants
# ---------------------------------------------------------------------------
HOST = "127.0.0.1"
ARM_PORT = 8010
HAND_PORT = 8000
TWIN_PORT = 8020

# ---------------------------------------------------------------------------
# Default model paths (relative to PROJECT_ROOT unless absolute)
# ---------------------------------------------------------------------------
DEFAULT_YOLO_MODEL = os.path.join(
    PROJECT_ROOT,
    "dependence", "yolo_world", "yolov8x-worldv2.pt",
)
DEFAULT_ANYGRASP_CHECKPOINT = os.path.join(
    PROJECT_ROOT,
    "dependence", "anygrasp_sdk", "checkpoint_detection.tar",
)

# ---------------------------------------------------------------------------
# Hand gesture configurations
# ---------------------------------------------------------------------------
HAND_CLOSE = [0, 0, 0, 460, 0, 0]
HAND_OPEN = [1000, 1000, 1000, 1000, 1000, 0]

# ---------------------------------------------------------------------------
# Named poses exposed from robot_config.json
# ---------------------------------------------------------------------------
NAMED_POSES = [
    "grasp1", "grasp2", "grasp3", "grasp4",
    "place1", "place2",
    "get_ready_to_handover_1st",
    "get_ready_to_handover_2nd",
    "handover_pose",
    "throw_to_trash_pose",
    "look_over_what_in_user_hand_pose",
    "desk_pose_1",
    "desk_pose_2",
    "desk_pose_3",
]


class Config(BaseConfig):
    """Loads and exposes all runtime configuration."""

    def __init__(self, config_path=None):
        if config_path is None:
            config_path = os.path.join(PROJECT_ROOT, "robot_config.json")
        self.config_path = config_path
        self.robot_config: dict = {}
        self.default_traj_js: dict = {}
        self.base_link_name: str = "base_link"
        self.camera_link_name: str = "cam_link_grasp"
        self.hand_effector_name: str = "L_hand_endeffector"
        self.arm_end_link_name: str = "Link7"
        self.reload()

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------
    def reload(self) -> None:
        """Re-read robot_config.json from disk."""
        with open(self.config_path, "r") as f:
            self.robot_config = json.load(f)

        self.default_traj_js = self.robot_config.get("default_traj_js", {})
        self.base_link_name = self.robot_config.get("base_link_name", self.base_link_name)
        self.camera_link_name = self.robot_config.get("camera_link_name", self.camera_link_name)
        self.hand_effector_name = self.robot_config.get("hand_effector_name", self.hand_effector_name)
        self.arm_end_link_name = self.robot_config.get("arm_end_link_name", self.arm_end_link_name)

    def get_pose(self, name):
        """Return a named joint-space pose dict (e.g. {"J1": ..., "J7": ...})."""
        # Check top-level first (handover, desk poses, etc.)
        pose = self.robot_config.get(name)
        if pose is not None and "J1" in pose:
            return pose
        # Then check inside default_traj_js
        return self.default_traj_js.get(name)

    def get_named_poses(self) -> dict:
        """Return all named poses (merged top-level + default_traj_js)."""
        poses = {}
        poses.update(self.default_traj_js)
        for key in NAMED_POSES:
            val = self.robot_config.get(key)
            if val is not None and isinstance(val, dict) and "J1" in val:
                poses[key] = val
        return poses
