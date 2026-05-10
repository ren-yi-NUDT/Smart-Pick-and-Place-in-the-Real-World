"""Inspire dexterous hand driver — wraps the existing HandClient."""

from core.hand import HandClient
from core.drivers.hand_driver import HandDriver


class InspireHandDriver(HandDriver):
    """Concrete driver for the Inspire dexterous hand via socket bridge."""

    def __init__(self, host: str, port: int, service_name: str, gestures: dict):
        self._client = HandClient(host, port, service_src=service_name)
        self._client._hand_config = {k: list(v) for k, v in gestures.items()}

    def connect(self) -> bool:
        return self._client.connect()

    def close_connection(self) -> None:
        self._client.close_connection()

    def open(self) -> dict:
        return self._client.open()

    def close(self) -> dict:
        return self._client.close()

    def get_state(self) -> dict:
        return self._client.get_state()

    def is_grasping(self) -> bool:
        return self._client.is_grasping()
