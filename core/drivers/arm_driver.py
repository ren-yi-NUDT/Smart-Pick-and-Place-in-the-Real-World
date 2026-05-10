"""Abstract base class for robot arm drivers."""

from abc import ABC, abstractmethod


class ArmDriver(ABC):
    """Hardware-agnostic interface for robot arm control."""

    @abstractmethod
    def connect(self) -> bool:
        ...

    @abstractmethod
    def close(self) -> None:
        ...

    @abstractmethod
    def move_to_named_pose(self, pose_dict: dict, speed: int = 30) -> bool:
        ...

    @abstractmethod
    def execute_trajectory(self, trajectory, speed: int = 20) -> bool:
        ...
