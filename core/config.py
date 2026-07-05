"""
Configuration loader and constants for Smart Pick-and-Place.

Usage:
    from core.config import Config
    cfg = Config()                       # loads robot_config.json from project root
    cfg = Config(config_path="/my/path")  # explicit path

Supports two config file formats:
  - **New format** (dual-arm): top-level keys ``arms``, ``shared``, ``dual_arm``.
  - **Legacy format** (single-arm): flat dict with ``default_traj_js`` at top level.

All existing code that accesses ``cfg.robot_config``, ``cfg.default_traj_js``,
``cfg.get_pose(name)``, etc. continues to work -- the new format transparently
maps those accesses to the ``left`` arm section.
"""

import json
import os

# ---------------------------------------------------------------------------
# Project root (the directory that contains robot_config.json)
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# Network constants  (kept for backward-compatibility; new code should read
# ports from Config.shared / Config.get_arm_config(...)["arm_port"])
# ---------------------------------------------------------------------------
HOST = "127.0.0.1"
ARM_PORT = 8010
HAND_PORT = 8000
TWIN_PORT = 8020          # left arm twin IK service
TWIN_PORT_RIGHT = 8021    # right arm twin IK service
DEFAULT_ANYGRASP_HOST = "127.0.0.1"
DEFAULT_ANYGRASP_PORT = 8030

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


class Config:
    """Loads and exposes all runtime configuration.

    Automatically detects the config file format (new dual-arm vs. legacy
    single-arm) and provides a uniform interface.
    """

    def __init__(self, config_path=None):
        if config_path is None:
            config_path = os.path.join(PROJECT_ROOT, "robot_config.json")
        self.config_path = config_path

        # Internal state
        self._raw: dict = {}               # the raw JSON dict
        self._is_new_format: bool = False  # True when "arms" key present
        self._arms: dict = {}              # {"left": {...}, "right": {...}}
        self._shared: dict = {}
        self._dual_arm: dict = {}

        # Backward-compat attributes (populated in reload)
        self.robot_config: dict = {}
        self.default_traj_js: dict = {}
        self.base_link_name: str = "base_link"
        self.camera_link_name: str = "cam_link_grasp"
        self.hand_effector_name: str = "L_hand_endeffector"
        self.arm_end_link_name: str = "Link7"

        self.reload()

    # ------------------------------------------------------------------
    # Internal format detection
    # ------------------------------------------------------------------
    def _detect_format(self, data: dict) -> bool:
        """Return True if *data* uses the new dual-arm format."""
        return "arms" in data

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------
    def reload(self) -> None:
        """Re-read robot_config.json from disk."""
        with open(self.config_path, "r") as f:
            self._raw = json.load(f)

        self._is_new_format = self._detect_format(self._raw)

        if self._is_new_format:
            self._arms = self._raw["arms"]
            self._shared = self._raw.get("shared", {})
            self._dual_arm = self._raw.get("dual_arm", {})

            # --- Build backward-compat views from the LEFT arm --------
            left = self._arms.get("left", {})
            self.default_traj_js = left.get("default_traj_js", {})
            self.base_link_name = left.get("base_link_name", self.base_link_name)
            self.hand_effector_name = left.get(
                "hand_effector_name", self.hand_effector_name
            )
            self.arm_end_link_name = left.get(
                "arm_end_link_name", self.arm_end_link_name
            )
            self.camera_link_name = self._shared.get(
                "camera_link_name", self.camera_link_name
            )

            # Synthesise a flat ``robot_config`` so that code doing
            # ``self.config.robot_config.get("handover_pose")`` still works.
            self.robot_config = {}
            self.robot_config["default_traj_js"] = self.default_traj_js
            self.robot_config["base_link_name"] = self.base_link_name
            self.robot_config["camera_link_name"] = self.camera_link_name
            self.robot_config["hand_effector_name"] = self.hand_effector_name
            self.robot_config["arm_end_link_name"] = self.arm_end_link_name
            for key in NAMED_POSES:
                val = left.get(key)
                if val is not None:
                    self.robot_config[key] = val
        else:
            # --- Legacy single-arm format -----------------------------
            self._arms = {}
            self._shared = {}
            self._dual_arm = {}

            self.robot_config = self._raw
            self.default_traj_js = self._raw.get("default_traj_js", {})
            self.base_link_name = self._raw.get(
                "base_link_name", self.base_link_name
            )
            self.camera_link_name = self._raw.get(
                "camera_link_name", self.camera_link_name
            )
            self.hand_effector_name = self._raw.get(
                "hand_effector_name", self.hand_effector_name
            )
            self.arm_end_link_name = self._raw.get(
                "arm_end_link_name", self.arm_end_link_name
            )

    # ------------------------------------------------------------------
    # New API: arm-scoped access
    # ------------------------------------------------------------------
    def get_arm_config(self, side: str) -> dict:
        """Return the full config dict for the requested arm (``"left"`` / ``"right"``).

        The returned dict contains keys like ``arm_port``, ``hand_port``,
        ``hand_type``, ``default_traj_js``, and all named poses.
        """
        if side not in self._arms:
            raise KeyError(
                f"Arm side '{side}' not found in config. "
                f"Available: {list(self._arms.keys())}"
            )
        return self._arms[side]

    @property
    def shared(self) -> dict:
        """Return the shared configuration section (host, camera_link_name, twin_port)."""
        return self._shared

    @property
    def dual_arm(self) -> dict:
        """Return the dual-arm configuration section."""
        return self._dual_arm

    def get_dual_arm_pose(self, name: str) -> dict:
        """Return a named entry from the ``dual_arm`` section.

        Example: ``cfg.get_dual_arm_pose("left_to_right_handover")``
        returns ``{"left_pose": {...}, "right_pose": {...}}``.
        """
        entry = self._dual_arm.get(name)
        if entry is None:
            raise KeyError(
                f"Dual-arm pose '{name}' not found in config. "
                f"Available: {list(self._dual_arm.keys())}"
            )
        return entry

    # ------------------------------------------------------------------
    # Backward-compatible API
    # ------------------------------------------------------------------
    def get_pose(self, name: str, side: str = "left"):
        """Return a named joint-space pose dict (e.g. {"J1": ..., "J7": ...}).

        By default queries the **left** arm for backward compatibility.
        Pass ``side="right"`` to query the right arm.
        """
        if self._is_new_format:
            arm_data = self._arms.get(side, {})
            pose = arm_data.get(name)
            if pose is not None and isinstance(pose, dict) and "J1" in pose:
                return pose
            traj_js = arm_data.get("default_traj_js", {})
            return traj_js.get(name)
        else:
            # Legacy format
            pose = self.robot_config.get(name)
            if pose is not None and "J1" in pose:
                return pose
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
