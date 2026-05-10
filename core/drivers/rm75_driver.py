"""RM-75 7-DOF arm driver — wraps the existing ArmClient."""

from core.arm import ArmClient
from core.drivers.arm_driver import ArmDriver


class RM75ArmDriver(ArmDriver):
    """Concrete driver for the RM-75 arm via the ROS socket bridge."""

    def __init__(self, host: str, port: int, service_name: str, joint_names: list):
        self._client = ArmClient(host, port, joint_names=joint_names)
        self._client.SERVICE_NAME = service_name

    def connect(self) -> bool:
        return self._client.connect()

    def close(self) -> None:
        self._client.close()

    def move_to_named_pose(self, pose_dict: dict, speed: int = 30) -> bool:
        return self._client.move_to_named_pose(pose_dict, speed=speed)

    def execute_trajectory(self, trajectory, speed: int = 20) -> bool:
        return self._client.execute_trajectory(trajectory, speed=speed)
