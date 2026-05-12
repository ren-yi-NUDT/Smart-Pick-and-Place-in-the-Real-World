"""
Backend factory — selects hardware implementations based on ROBOT_BACKEND.

Usage:
    ROBOT_BACKEND=rm65   python run_skill.py pick_and_place   (default)
    ROBOT_BACKEND=igrape python run_skill.py pick_and_place
"""

import os

BACKEND = os.environ.get("ROBOT_BACKEND", "rm65")


def create_config(config_path=None, save_path="./log"):
    if BACKEND == "igrape":
        from core.backends.igrape.config import IgrapeConfig
        return IgrapeConfig()
    from core.config import Config
    return Config(config_path)


def create_arm(**kwargs):
    if BACKEND == "igrape":
        from core.backends.igrape.arm import IgrapeArm
        return IgrapeArm(**kwargs)
    from core.config import HOST, ARM_PORT
    from core.arm import ArmClient
    return ArmClient(kwargs.get("host", HOST), kwargs.get("port", ARM_PORT))


def create_hand(**kwargs):
    if BACKEND == "igrape":
        from core.backends.igrape.hand import IgrapeHand
        return IgrapeHand(**kwargs)
    from core.config import HOST, HAND_PORT
    from core.hand import HandClient
    return HandClient(kwargs.get("host", HOST), kwargs.get("port", HAND_PORT))


def create_twin(**kwargs):
    if BACKEND == "igrape":
        from core.backends.igrape.twin_client import IgrapeTwinClient
        return IgrapeTwinClient(**kwargs)
    from core.config import HOST, TWIN_PORT
    from core.twin_client import TwinClient
    return TwinClient(kwargs.get("host", HOST), kwargs.get("port", TWIN_PORT))


def create_camera(save_path="./log", **kwargs):
    if BACKEND == "igrape":
        from core.backends.igrape.camera import IgrapeCamera
        return IgrapeCamera(save_path=save_path, **kwargs)
    from core.camera import RealSenseCapture
    return RealSenseCapture(width=640, height=480, fps=30, save_path=save_path)


def create_transforms():
    if BACKEND == "igrape":
        from core.backends.igrape.transforms import IgrapeTransforms
        return IgrapeTransforms()
    from core.transforms import TransformationUtil
    return TransformationUtil()


def create_perception(**kwargs):
    if BACKEND == "igrape":
        from core.backends.igrape.perception import IgrapePerception
        return IgrapePerception(**kwargs)
    from core.perception import Perception
    from core.config import DEFAULT_YOLO_MODEL, DEFAULT_ANYGRASP_CHECKPOINT
    return Perception(
        yolo_model_path=kwargs.get("yolo_model_path", DEFAULT_YOLO_MODEL),
        anygrasp_checkpoint=kwargs.get("anygrasp_checkpoint", DEFAULT_ANYGRASP_CHECKPOINT),
    )


def create_vlm(**kwargs):
    if BACKEND == "igrape":
        from core.backends.igrape.vlm import IgrapeVLM
        return IgrapeVLM(**kwargs)
    from core.vlm import VLMClient
    return VLMClient(**kwargs)
