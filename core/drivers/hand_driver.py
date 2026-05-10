"""Abstract base class for robot hand / gripper drivers."""

from abc import ABC, abstractmethod


class HandDriver(ABC):
    """Hardware-agnostic interface for hand / gripper control."""

    @abstractmethod
    def connect(self) -> bool:
        ...

    @abstractmethod
    def close_connection(self) -> None:
        ...

    @abstractmethod
    def open(self) -> dict:
        ...

    @abstractmethod
    def close(self) -> dict:
        ...

    @abstractmethod
    def get_state(self) -> dict:
        ...

    @abstractmethod
    def is_grasping(self) -> bool:
        ...
