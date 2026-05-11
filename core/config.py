"""
Configuration loader and constants for Smart Pick-and-Place.

Usage:
    from core.config import Config
    cfg = Config()                       # loads robot_config.json from project root
    cfg = Config(config_path="/my/path")  # explicit path
"""

import json
import math
import os

# ---------------------------------------------------------------------------
# Project root (the directory that contains robot_config.json)
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# Default profile path
# ---------------------------------------------------------------------------
DEFAULT_PROFILE_PATH = os.path.join(PROJECT_ROOT, "robot_profile.json")

# ---------------------------------------------------------------------------
# Fallback constants (used when robot_profile.json is absent)
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
# Hand gesture configurations (fallback)
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


def _fallback_profile():
    """Synthesize a profile from the legacy constants (RM-75 defaults)."""
    return {
        "arm": {
            "driver": "rm75",
            "service_name": "/right_arm/movement_control",
            "struct_name": "left_arm",
            "num_joints": 7,
            "joint_names": ["J1", "J2", "J3", "J4", "J5", "J6", "J7"],
            "host": HOST,
            "port": ARM_PORT,
        },
        "hand": {
            "driver": "inspire",
            "service_name": "/left_hand/movement_control",
            "gestures": {
                "close": list(HAND_CLOSE),
                "open": list(HAND_OPEN),
            },
            "host": HOST,
            "port": HAND_PORT,
        },
        "twin": {
            "host": HOST,
            "port": TWIN_PORT,
            "urdf_path": os.path.join(
                PROJECT_ROOT,
                "dependence", "smart_pick_and_place_ws", "src", "rm_description",
                "urdf", "SingleArm", "easy_single_arm_bullet.urdf",
            ),
            "robot_config_path": os.path.join(
                PROJECT_ROOT,
                "dependence", "smart_pick_and_place_ws", "src", "rm_description",
                "urdf", "SingleArm", "robot_config.json",
            ),
        },
        "frames": {
            "base_link": "base_link",
            "camera_link": "cam_link_grasp",
            "hand_effector": "L_hand_endeffector",
            "arm_end_link": "Link7",
        },
        "camera": {
            "type": "realsense",
        },
        "perception": {
            "anygrasp_ws_url": "",
        },
    }


class Config:
    """Loads and exposes all runtime configuration."""

    def __init__(self, config_path=None, profile_path=None):
        if config_path is None:
            config_path = os.path.join(PROJECT_ROOT, "robot_config.json")
        self.config_path = config_path
        self.profile_path = profile_path

        self.robot_config: dict = {}
        self.default_traj_js: dict = {}
        self.profile: dict = {}

        # Frame defaults (overridden by profile / robot_config)
        self.base_link_name: str = "base_link"
        self.camera_link_name: str = "cam_link_grasp"
        self.hand_effector_name: str = "L_hand_endeffector"
        self.arm_end_link_name: str = "Link7"

        self.reload()

    # ------------------------------------------------------------------
    # Profile helpers
    # ------------------------------------------------------------------
    @property
    def arm_joint_names(self) -> list:
        """Ordered joint name list, e.g. ['J1','J2',...,'J7']."""
        return self.profile["arm"]["joint_names"]

    @property
    def arm_num_joints(self) -> int:
        return self.profile["arm"]["num_joints"]

    @property
    def arm_struct_name(self) -> str:
        """Struct name for twin service requests."""
        return self.profile["arm"]["struct_name"]

    def pose_to_list(self, pose_dict: dict) -> list:
        """Convert a pose dict to an ordered list using profile joint names."""
        return [pose_dict[name] for name in self.arm_joint_names]

    def list_to_pose(self, values: list) -> dict:
        """Convert an ordered list of joint values to a dict."""
        return dict(zip(self.arm_joint_names, values))

    def get_default_js_rad(self, pose_name: str) -> list:
        """Get a named pose as a list of values in radians."""
        pose = self.get_pose(pose_name)
        if pose is None:
            raise KeyError(f"Pose '{pose_name}' not found in config")
        return [v / 180.0 * math.pi for v in self.pose_to_list(pose)]

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------
    def reload(self) -> None:
        """Re-read robot_config.json and robot_profile.json from disk."""
        with open(self.config_path, "r") as f:
            self.robot_config = json.load(f)

        self.default_traj_js = self.robot_config.get("default_traj_js", {})

        # Load profile (fallback to synthesized RM-75 profile)
        if self.profile_path is None:
            self.profile_path = DEFAULT_PROFILE_PATH
        if os.path.isfile(self.profile_path):
            with open(self.profile_path, "r") as f:
                self.profile = json.load(f)
        else:
            self.profile = _fallback_profile()

        # Frame names: profile first, then robot_config override, then defaults
        frames = self.profile.get("frames", {})
        self.base_link_name = (
            self.robot_config.get("base_link_name", frames.get("base_link", self.base_link_name))
        )
        self.camera_link_name = (
            self.robot_config.get("camera_link_name", frames.get("camera_link", self.camera_link_name))
        )
        self.hand_effector_name = (
            self.robot_config.get("hand_effector_name", frames.get("hand_effector", self.hand_effector_name))
        )
        self.arm_end_link_name = (
            self.robot_config.get("arm_end_link_name", frames.get("arm_end_link", self.arm_end_link_name))
        )

    def get_pose(self, name):
        """Return a named joint-space pose dict (e.g. {"J1": ..., "J7": ...})."""
        first_joint = self.arm_joint_names[0]
        pose = self.robot_config.get(name)
        if pose is not None and first_joint in pose:
            return pose
        return self.default_traj_js.get(name)

    def get_named_poses(self) -> dict:
        """Return all named poses (merged top-level + default_traj_js)."""
        first_joint = self.arm_joint_names[0]
        poses = {}
        poses.update(self.default_traj_js)
        for key in NAMED_POSES:
            val = self.robot_config.get(key)
            if val is not None and isinstance(val, dict) and first_joint in val:
                poses[key] = val
        return poses
