"""
Configuration loader and constants for Smart Pick-and-Place.

Usage:
    from core.config import Config
    cfg = Config()                       # loads robot_config.json from project root
    cfg = Config(config_path="/my/path")  # explicit path
"""

import json
import os

from core.abc import BaseConfig

# ---------------------------------------------------------------------------
# Project root (the directory that contains robot_config.json)
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# Network constants
# ---------------------------------------------------------------------------
HOST = "127.0.0.1"
ARM_PORT = 8010
GRIPPER_PORT = 8001  # Robotiq 85 夹爪服务 (dependence/gripper-programming/server.py)
TWIN_PORT = 8020

# ---------------------------------------------------------------------------
# Default model paths (relative to PROJECT_ROOT unless absolute)
# ---------------------------------------------------------------------------
DEFAULT_YOLO_MODEL = os.path.join(
    PROJECT_ROOT,
    "dependence", "yolo_world", "yolov8x-worldv2.pt",
)
DEFAULT_ANYGRASP_CHECKPOINT = os.path.join(
    PROJECT_ROOT,
    "dependence", "anygrasp_sdk", "checkpoint_detection.tar",
)

# ---------------------------------------------------------------------------
# YOLO-World 置信度阈值
#
# 实测这台相机在当前光照下分数极低: 盘子 0.096~0.29, 黄瓜 0.012 (框是准的,
# 分数是噪声级的)。抓取侧因此压到 0.05 —— 阈值高了目标直接漏检。
# 容器侧风险不同: 假阳性会把机械臂导向一个不存在的容器, 所以留得高一些。
# 光照修好后 (采集帧平均亮度 >60) 两边都可以往上调。
# ---------------------------------------------------------------------------
YOLO_CONF_GRASP = 0.05
YOLO_CONF_CONTAINER = 0.10

# 容器框的最小边长 (px)。框被画面边缘切掉、或者小到只剩一条边时, 框中心不是容器中心,
# 算出来的放置点会偏出容器 —— 实测 grasp3 看蓝盘子只框到 21px 高的一条, 火龙果就放在
# 盘子旁边的桌上。见 core/perception.py:_well_framed。
MIN_CONTAINER_BOX_PX = 30

# look_around 建场景地图时用的阈值。比抓取侧高: 扫描一次要给 15 个类名, 阈值太低
# 会灌进一堆假框写进地图 (实测 0.05 时多了 box/bottle 三个假阳性)。
YOLO_CONF_SCAN = 0.20

# ---------------------------------------------------------------------------
# 相机白平衡
#
# 原来写死 5500K。房间灯是暖光 (~3500K), 设成 5500K 等于告诉相机"光很白"于是不做
# 补偿, 整幅图偏橙。后果不只是不好看 —— 实测粉盘子和蓝盘子在这个设置下色相**都是
# 6°**, 蓝盘子的蓝完全被抹掉了, 于是 YOLO 的颜色词只能靠语言先验瞎猜。
# 改成 3500K 后: 粉盘子 354° / 蓝盘子 204° / 木桌面 24°, 三个干净分开。
# 换灯或者换房间后要重标这个值 (相机 WB 范围 2800-6500)。
# ---------------------------------------------------------------------------
CAMERA_WHITE_BALANCE = 3500
CAMERA_AUTO_WHITE_BALANCE = False

# RealSense 补洞滤波模式。0=从左邻居填 1=从右邻居填 2=按邻域深度填。
# 只能用 2: 实测 mode 1 会把野值 (22.5m) 铺满整片物体区域, 见 core/camera.py。
HOLE_FILL_MODE = 2

# ---------------------------------------------------------------------------
# 被物体挡住的深度洞 (D455 最小可测距离 ~286mm, 相机离桌面只有 ~30cm, 有高度的
# 物体整个落在死区里) 补成一个穹顶而不是桌面。见 core/camera.py:dome_fill_box。
# ---------------------------------------------------------------------------
# 小于这个像素数的洞当成噪点/细缝, 不补 (实测橙子的洞 ~10400px)。
MIN_OCCLUDED_HOLE_PX = 150
# 穹顶高度上限 (米)。挡住一个大洞的也可能是阴影之类, 别顶出一个荒唐的尖。
MAX_OCCLUDED_HEIGHT_M = 0.15

# ---------------------------------------------------------------------------
# 采集帧最低平均亮度 (0-255)
#
# 曝光/增益顶格 (10000/128) 还是欠曝时, 下游 YOLO 和 AnyGrasp 都在废图上跑,
# 但日志里看不出任何异常。实测当前环境 mean≈18, 正常光照应 >60。
# ---------------------------------------------------------------------------
MIN_FRAME_BRIGHTNESS = 40

# ---------------------------------------------------------------------------
# Named poses exposed from robot_config.json
# ---------------------------------------------------------------------------
NAMED_POSES = [
    "grasp1", "grasp2", "grasp3", "grasp4",
    "place1", "place2",
    "get_ready_to_handover_1st",
    "get_ready_to_handover_2nd",
    "handover_pose",
    "throw_to_trash_pose",
    "look_over_what_in_user_hand_pose",
    "desk_pose_1",
    "desk_pose_2",
    "desk_pose_3",
]


def _fold180(v: float) -> float:
    """折进 (-180, 180]。"""
    while v > 180.0:
        v -= 360.0
    while v <= -180.0:
        v += 360.0
    return v


def wrist_headroom_pose(pose: dict) -> dict:
    """观察位姿换成 ZYZ 等价解: 物理位姿一模一样, 但腕部有余量。

    实测 (2026-10-04) grasp1-4 录下来 J4 ≈ -164~-172.5°, 离真机 ±177.96° 限位只剩 ~9°。
    腕部没有余量 = "再转一点就出限位", 于是任何需要改变夹爪朝向的抓取候选都会被 IK 逼到
    ZYZ 翻转分支上; 而翻转分支要 J4 从 -169° 走到 +172°, 只能穿过 ±180° 的限位死区 ——
    走不过去, 孪生的 BRANCH_JUMP 保护因此**正确**地把候选判成不可用 (twin.py:530)。

    实测同一个橙子, 同一个位姿: 原始 grasp1 (J4=-169°) 7 个候选只有 2 个能生成轨迹;
    换成翻转解 (J4=+11°) 10 个里 9 个能。抓取失败的大头就在这。

    翻转是 (J4+180, -J5, J6+180), 末端位姿不变 —— 实测位置差 0.04mm / 姿态差 0.006°,
    相机是 link_6 的固定子节点 (mount_camera.py: parent_frame='link_6'), 所以画面也不变。
    只在 J4 贴限位时才换, 所以对已经有余量的位姿是幂等的。
    """
    out = dict(pose)
    if abs(out["J4"]) <= 90.0:
        return out
    out["J4"] = _fold180(out["J4"] + 180.0)
    out["J5"] = -out["J5"]
    out["J6"] = _fold180(out["J6"] + 180.0)
    return out


class Config(BaseConfig):
    """Loads and exposes all runtime configuration."""

    def __init__(self, config_path=None):
        if config_path is None:
            config_path = os.path.join(PROJECT_ROOT, "robot_config.json")
        self.config_path = config_path
        self.robot_config: dict = {}
        self.default_traj_js: dict = {}
        self.base_link_name: str = "base_link"
        self.camera_link_name: str = "cam_link_grasp"
        self.hand_effector_name: str = "gripper_endeffector"
        self.arm_end_link_name: str = "link_6"
        self.reload()

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------
    def reload(self) -> None:
        """Re-read robot_config.json from disk."""
        with open(self.config_path, "r") as f:
            self.robot_config = json.load(f)

        self.default_traj_js = self.robot_config.get("default_traj_js", {})
        # 抓取观察位姿统一换成腕部有余量的 ZYZ 等价解。见 wrist_headroom_pose:
        # 录下来的 J4 贴着 ±178° 限位, 不改的话任何改朝向的抓取候选都会被孪生判成不可达。
        # 只动 grasp* 观察位姿 —— 递送/扔垃圾/放桌面那些位姿不需要腕部余量, 别去动它们。
        for _key, _pose in self.default_traj_js.items():
            if _key.startswith("grasp") and isinstance(_pose, dict) and "J4" in _pose:
                self.default_traj_js[_key] = wrist_headroom_pose(_pose)
        self.base_link_name = self.robot_config.get("base_link_name", self.base_link_name)
        self.camera_link_name = self.robot_config.get("camera_link_name", self.camera_link_name)
        self.hand_effector_name = self.robot_config.get("hand_effector_name", self.hand_effector_name)
        self.arm_end_link_name = self.robot_config.get("arm_end_link_name", self.arm_end_link_name)

    def get_pose(self, name):
        """Return a named six-joint pose dict (e.g. {"J1": ..., "J6": ...})."""
        # Check top-level first (handover, desk poses, etc.)
        pose = self.robot_config.get(name)
        if pose is not None and "J1" in pose:
            return pose
        # Then check inside default_traj_js
        return self.default_traj_js.get(name)

    def get_named_poses(self) -> dict:
        """Return all named poses (merged top-level + default_traj_js)."""
        poses = {}
        poses.update(self.default_traj_js)
        for key in NAMED_POSES:
            val = self.robot_config.get(key)
            if val is not None and isinstance(val, dict) and "J1" in val:
                poses[key] = val
        return poses
