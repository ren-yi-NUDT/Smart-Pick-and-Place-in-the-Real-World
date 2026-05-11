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

# Lazy-loaded drivers that require rclpy (not always available on RM-75 host)
_TIANYI_ARM_DRIVER_CLASS = None
_TIANYI_HAND_DRIVER_CLASS = None


def _get_tianyi_arm_driver():
    global _TIANYI_ARM_DRIVER_CLASS
    if _TIANYI_ARM_DRIVER_CLASS is None:
        from core.drivers.tianyi_arm_driver import TianyiArmDriver
        _TIANYI_ARM_DRIVER_CLASS = TianyiArmDriver
    return _TIANYI_ARM_DRIVER_CLASS


def _get_tianyi_hand_driver():
    global _TIANYI_HAND_DRIVER_CLASS
    if _TIANYI_HAND_DRIVER_CLASS is None:
        from core.drivers.tianyi_hand_driver import TianyiHandDriver
        _TIANYI_HAND_DRIVER_CLASS = TianyiHandDriver
    return _TIANYI_HAND_DRIVER_CLASS


def create_arm_driver(profile_arm: dict) -> ArmDriver:
    """Instantiate an arm driver from the 'arm' section of robot_profile.json."""
    driver_name = profile_arm["driver"]

    if driver_name == "tianyi":
        cls = _get_tianyi_arm_driver()
        return cls(
            host=profile_arm.get("host"),
            port=profile_arm.get("port"),
            service_name=profile_arm.get("service_name"),
            joint_names=profile_arm.get("joint_names"),
        )

    cls = ARM_DRIVERS.get(driver_name)
    if cls is None:
        raise KeyError(f"Unknown arm driver: {driver_name}")
    return cls(
        host=profile_arm["host"],
        port=profile_arm["port"],
        service_name=profile_arm["service_name"],
        joint_names=profile_arm["joint_names"],
    )


def create_hand_driver(profile_hand: dict) -> HandDriver:
    """Instantiate a hand driver from the 'hand' section of robot_profile.json."""
    driver_name = profile_hand["driver"]

    if driver_name == "tianyi_inspire":
        cls = _get_tianyi_hand_driver()
        return cls(
            host=profile_hand.get("host"),
            port=profile_hand.get("port"),
            service_name=profile_hand.get("service_name"),
            gestures=profile_hand.get("gestures"),
        )

    cls = HAND_DRIVERS.get(driver_name)
    if cls is None:
        raise KeyError(f"Unknown hand driver: {driver_name}")
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
