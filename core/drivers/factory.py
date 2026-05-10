"""Driver factory — creates hardware drivers from profile config."""

from core.drivers.arm_driver import ArmDriver
from core.drivers.hand_driver import HandDriver
from core.drivers.rm75_driver import RM75ArmDriver
from core.drivers.inspire_driver import InspireHandDriver

ARM_DRIVERS = {
    "rm75": RM75ArmDriver,
}

HAND_DRIVERS = {
    "inspire": InspireHandDriver,
}


def create_arm_driver(profile_arm: dict) -> ArmDriver:
    """Instantiate an arm driver from the 'arm' section of robot_profile.json."""
    cls = ARM_DRIVERS[profile_arm["driver"]]
    return cls(
        host=profile_arm["host"],
        port=profile_arm["port"],
        service_name=profile_arm["service_name"],
        joint_names=profile_arm["joint_names"],
    )


def create_hand_driver(profile_hand: dict) -> HandDriver:
    """Instantiate a hand driver from the 'hand' section of robot_profile.json."""
    cls = HAND_DRIVERS[profile_hand["driver"]]
    return cls(
        host=profile_hand["host"],
        port=profile_hand["port"],
        service_name=profile_hand["service_name"],
        gestures=profile_hand["gestures"],
    )


def register_arm_driver(name: str, cls):
    """Register a new arm driver implementation."""
    ARM_DRIVERS[name] = cls


def register_hand_driver(name: str, cls):
    """Register a new hand driver implementation."""
    HAND_DRIVERS[name] = cls
