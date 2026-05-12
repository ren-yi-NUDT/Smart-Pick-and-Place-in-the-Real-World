"""
Abstract base classes for the Skill-DB hardware abstraction layer.

Each ABC defines the minimum interface that a robot backend must implement.
Skills only depend on these interfaces — switching robots means swapping
the backend, not the skill code.
"""

from abc import ABC, abstractmethod


class BaseConfig(ABC):
    """Configuration loader and named-pose registry."""

    base_link_name: str
    camera_link_name: str
    hand_effector_name: str
    arm_end_link_name: str
    default_traj_js: dict
    robot_config: dict

    @abstractmethod
    def get_pose(self, name: str):
        """Return a named joint-space pose dict (e.g. {"J1": deg, ..., "J7": deg}).

        Returns None if the pose is not found.
        """

    @abstractmethod
    def get_named_poses(self) -> dict:
        """Return all named poses as a dict."""

    @abstractmethod
    def reload(self) -> None:
        """Re-read configuration from disk."""


class BaseArm(ABC):
    """Robot arm controller."""

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def reset_cmd(self) -> None:
        """Clear the pending command queue."""

    @abstractmethod
    def start_cmd(self) -> None: ...

    @abstractmethod
    def add_js_cmd(self, joint_dict: dict, speed: int = 5, block: bool = True) -> None:
        """Append a joint-space command. joint_dict: {"J1": deg, ..., "J7": deg}."""

    @abstractmethod
    def add_ee_cmd(self, ee_trajectory, speed: int = 5, block: bool = True) -> None: ...

    @abstractmethod
    def send_cmds(self) -> dict: ...

    @abstractmethod
    def move_to_named_pose(self, pose_dict: dict, speed: int = 30) -> bool: ...

    @abstractmethod
    def execute_trajectory(self, trajectory, speed: int = 20) -> bool: ...


class BaseHand(ABC):
    """Dexterous hand / gripper controller."""

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def close_connection(self) -> None: ...

    @abstractmethod
    def open(self) -> dict: ...

    @abstractmethod
    def close(self) -> dict: ...

    @abstractmethod
    def get_state(self) -> dict: ...

    @abstractmethod
    def is_grasping(self) -> bool: ...


class BaseCamera(ABC):
    """RGB-D camera capture."""

    @abstractmethod
    def get_rgbd(self, idx: int = 0):
        """Capture a single RGB-D frame.

        Returns
        -------
        rgb : np.ndarray (H, W, 3) uint8
        depth : np.ndarray (H, W) uint16
        """


class BaseTwinClient(ABC):
    """Digital-twin / trajectory inference client."""

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def call_service(self, srv_type: str, cnfg: dict) -> dict: ...

    @abstractmethod
    def generate_trajectory2(self, cnfg: dict) -> dict: ...


class BaseTransforms(ABC):
    """Coordinate transform lookup."""

    @abstractmethod
    def get_transform_from_frame_to_frame(self, from_frame: str, to_frame: str):
        """Look up the homogeneous transform between two frames.

        Returns
        -------
        translation_matrix : np.ndarray (4, 4)
        transformation_euler : list [tx, ty, tz, rx, ry, rz] (radians)
        transformation_quat : list [tx, ty, tz, qx, qy, qz, qw]
        """


class BasePerception(ABC):
    """Object detection and grasp pose estimation."""

    @abstractmethod
    def detect_objects(self, image, class_names, conf: float = 0.2) -> list: ...

    @abstractmethod
    def detect_grasps(self, rgb, depth, model: str = "rs_right") -> list: ...

    @abstractmethod
    def filter_grasps_by_detection(self, anygrasp_pose, image, class_name: str,
                                   return_label: bool = False, vis: bool = True) -> list: ...

    @abstractmethod
    def detect_placement_position(self, class_name: str, image, depth,
                                  cam_type: str = "right", vis: bool = False): ...


class BaseVLM(ABC):
    """Vision-language model client."""

    @abstractmethod
    def analyze(self, rgb_image, prompt: str = None,
                max_tokens: int = 1024, temperature: float = 0.7): ...
