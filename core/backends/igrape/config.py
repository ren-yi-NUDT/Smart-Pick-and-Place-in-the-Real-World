"""
Igrape-bot3 configuration adapter.

Loads igrape_config.json (pose name mapping, frame names, service URLs)
and Igrape's body/configs/actions.json, converting motor-ID radian poses
to the Skill-DB J1-J7 degree format.
"""

import json
import os

from core.abc import BaseConfig
from core.backends.igrape._joint_map import igrape_to_skill

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


class IgrapeConfig(BaseConfig):
    """Drop-in replacement for core.config.Config backed by Igrape configs."""

    def __init__(self):
        self.config_path = os.path.join(_PROJECT_ROOT, "igrape_config.json")

        with open(self.config_path, "r") as f:
            self._igrape_cfg = json.load(f)

        igrape_root = self._igrape_cfg["igrape_root"]
        actions_path = os.path.join(igrape_root, "body", "configs", "actions.json")
        with open(actions_path, "r") as f:
            self._actions = json.load(f)

        self._pose_map = self._igrape_cfg.get("pose_map", {})
        self._reverse_pose_map = {v: k for k, v in self._pose_map.items()}

        frames = self._igrape_cfg.get("frame_names", {})
        self.base_link_name = frames.get("base_link", "base")
        self.camera_link_name = frames.get("camera_link", "camera_link")
        self.hand_effector_name = frames.get("hand_effector", "right_hand_endeffector")
        self.arm_end_link_name = frames.get("arm_end_link", "Link7")

        self.robot_config = {}
        self.default_traj_js = {}
        self._build_compat_poses()

    def _build_compat_poses(self):
        """Convert Igrape {'21': rad, ...} poses -> Skill-DB {'J1': deg, ...}."""
        for skill_name, igrape_name in self._pose_map.items():
            raw = self._actions.get(igrape_name)
            if raw and isinstance(raw, dict):
                converted = igrape_to_skill(raw)
                if converted:
                    self.default_traj_js[skill_name] = converted

        # Also expose poses by their Igrape names (for direct access)
        for igrape_name, raw in self._actions.items():
            if isinstance(raw, dict):
                converted = igrape_to_skill(raw)
                if converted:
                    self.robot_config[igrape_name] = converted

    def get_pose(self, name):
        """Return a named pose dict {'J1': deg, ..., 'J7': deg}.

        Search order matches CLAUDE.md: top-level (robot_config) first,
        then default_traj_js.
        """
        # Check top-level (all Igrape actions) first
        pose = self.robot_config.get(name)
        if pose is not None:
            return pose
        # Then check mapped poses (default_traj_js)
        return self.default_traj_js.get(name)

    def get_named_poses(self) -> dict:
        return dict(self.default_traj_js)

    def reload(self):
        self.__init__()
