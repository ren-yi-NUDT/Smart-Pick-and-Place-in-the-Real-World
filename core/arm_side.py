"""
ArmSide -- per-arm context for a dual-arm robot system.

Each side (left / right) holds its own ArmClient and either a
HandClient (dexterous) or GripperClient, all lazily connected.
"""

from termcolor import cprint

from core.arm import ArmClient
from core.hand import HandClient
from core.gripper import GripperClient


class ArmSide:
    """Encapsulates one arm + its end-effector and configuration."""

    def __init__(self, side: str, arm_config: dict, host: str = "127.0.0.1"):
        """
        Args:
            side: "left" or "right"
            arm_config: per-arm config dict containing:
                arm_port, hand_port, hand_type ("dexterous"|"gripper"),
                base_link_name, hand_effector_name, arm_end_link_name,
                camera_link_name (optional), default_traj_js, and
                top-level special poses (handover_pose, etc.)
            host: TCP host for arm/hand services.
        """
        self.side = side
        self._arm_config = arm_config
        self._host = host
        self._arm = None
        self._hand = None

    # ------------------------------------------------------------------
    # Lazy-loaded clients
    # ------------------------------------------------------------------
    @property
    def arm(self) -> ArmClient:
        """ArmClient, created and connected on first access."""
        if self._arm is None:
            port = self._arm_config["arm_port"]
            self._arm = ArmClient(host=self._host, port=port)
            self._arm.connect()
            cprint(f"[ArmSide:{self.side}] ArmClient connected on :{port}", "green")
        return self._arm

    @property
    def hand(self):
        """HandClient or GripperClient, chosen by *hand_type*, lazily connected."""
        if self._hand is None:
            hand_type = self._arm_config.get("hand_type", "dexterous")
            port = self._arm_config["hand_port"]
            if hand_type == "gripper":
                src = f"/{self.side}_gripper/movement_control"
                self._hand = GripperClient(host=self._host, port=port, src=src)
            else:
                self._hand = HandClient(host=self._host, port=port)
            self._hand.connect()
            cprint(
                f"[ArmSide:{self.side}] {type(self._hand).__name__} connected on :{port}",
                "green",
            )
        return self._hand

    # ------------------------------------------------------------------
    # Pose lookup
    # ------------------------------------------------------------------
    def get_pose(self, name: str) -> dict:
        """Look up a named pose from this arm's config.

        Search order:
          1. Top-level key in arm_config (must be a dict containing "J1").
          2. Inside arm_config["default_traj_js"].
        Raises KeyError if not found.
        """
        candidate = self._arm_config.get(name)
        if isinstance(candidate, dict) and "J1" in candidate:
            return candidate
        traj_poses = self._arm_config.get("default_traj_js", {})
        if name in traj_poses:
            return traj_poses[name]
        raise KeyError(
            f"[ArmSide:{self.side}] Pose '{name}' not found in config"
        )

    # ------------------------------------------------------------------
    # Convenience properties
    # ------------------------------------------------------------------
    @property
    def base_link_name(self) -> str:
        return self._arm_config["base_link_name"]

    @property
    def hand_effector_name(self) -> str:
        return self._arm_config["hand_effector_name"]

    @property
    def arm_end_link_name(self) -> str:
        return self._arm_config["arm_end_link_name"]

    @property
    def camera_link_name(self) -> str:
        return self._arm_config.get("camera_link_name", "cam_link_grasp")
