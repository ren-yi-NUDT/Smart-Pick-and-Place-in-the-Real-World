"""Backend factory for the single RM65 robot."""

BACKEND = "rm65"


def create_config(config_path=None, save_path="./log"):
    from core.config import Config
    return Config(config_path)


def create_arm(**kwargs):
    from core.config import HOST, ARM_PORT
    from core.arm import ArmClient
    return ArmClient(kwargs.get("host", HOST), kwargs.get("port", ARM_PORT))


def create_hand(**kwargs):
    from core.config import HOST, GRIPPER_PORT
    from core.gripper import GripperClient
    # 真机末端为 Robotiq 2F-85 夹爪 (TCP :8001 -> Modbus RTU)
    return GripperClient(kwargs.get("host", HOST), kwargs.get("port", GRIPPER_PORT))


def create_twin(**kwargs):
    from core.config import HOST, TWIN_PORT
    from core.twin_client import TwinClient
    return TwinClient(kwargs.get("host", HOST), kwargs.get("port", TWIN_PORT))


def create_camera(save_path="./log", **kwargs):
    from core.camera import RealSenseCapture
    return RealSenseCapture(width=640, height=480, fps=30, save_path=save_path)


def create_transforms():
    from core.transforms import TransformationUtil
    return TransformationUtil()


def create_perception(**kwargs):
    from core.perception import Perception
    from core.config import DEFAULT_YOLO_MODEL, DEFAULT_ANYGRASP_CHECKPOINT
    return Perception(
        yolo_model_path=kwargs.get("yolo_model_path", DEFAULT_YOLO_MODEL),
        anygrasp_checkpoint=kwargs.get("anygrasp_checkpoint", DEFAULT_ANYGRASP_CHECKPOINT),
    )


def create_vlm(**kwargs):
    from core.vlm import VLMClient
    return VLMClient(**kwargs)
