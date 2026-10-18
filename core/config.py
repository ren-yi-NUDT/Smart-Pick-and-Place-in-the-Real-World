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
    "dependence", "yolo_world", "yoloe-26x-seg.pt",
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
#
# 2026-10-07 再压到 0.03: 火龙果 (pitaya) 在 grasp3 上框得又准又正 (162x155px,
# 完整落在画面里), 分数只有 0.045, 卡在 0.05 下面被判"没检出目标"。而它唯一分数高
# 的那一次 (grasp2, 0.645) 框被画面右沿切掉 (x2=637/640), 那一帧 AnyGrasp 一个落在
# 果子上的候选都给不出来 (掠射角, 点云立不起来)。两个视角一个"看得见抓不着"、
# 一个"抓得着看不见", 结果就是火龙果根本抓不了。
# 抓取侧假阳性并非零代价: 2026-10-10 黄瓜回合里, 粉色盘子上出现 0.05 分的幻影
# "banana" 框, AnyGrasp 候选落进框内被选中, 空抓还撞了盘子。提到 0.10 后 pitaya
# (0.045) 依然检不出 —— 但那是 AnyGrasp 候选质量问题, 靠压阈值救不回来, 别再
# 为它把阈值压回幻影区。容器侧不动。
# ---------------------------------------------------------------------------
YOLO_CONF_GRASP = 0.10
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
# 桌面在基座标系里的高度 (米)。
#
# 诊断工具 (tools/diagnose_grasp.py) 拿它当"接触点离桌面多高"的基准。
# 实测 (2026-10-07) 物体框外一圈桌面点云的中位 z: 0.009 (火龙果旁) / 0.013 (黄瓜旁)。
# 重新标定底座、换桌面或换房间后要重测: 把一个已知高度的方块放桌上, 看它的接触点 z。
# ---------------------------------------------------------------------------
TABLE_Z_M = 0.010

# ---------------------------------------------------------------------------
# 场景地图的有效期 (秒)。超过这个时间没重扫, pick_and_place 就拒绝开工, 让人先跑
# look_around —— 拿一张过期地图去够东西比不干活更糟 (会够到空处或撞到别的东西)。
# 默认 300s 是拍的, 真机上要试多久合适。
# ---------------------------------------------------------------------------
SCENE_MAP_TTL_S = 600

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


# ---------------------------------------------------------------------------
# 真机关节限位。唯一来源: dependence/twin_inference/robot.py:wrap_joint_limits 的注释。
# 注意 J4 的行程 355.92° 比整圈少 4°, 所以 ±180 附近有 4° 的**不可达缺口**。
# ---------------------------------------------------------------------------
LIMITS = {"J1": 177.96, "J2": 177.96, "J3": 177.96,
          "J4": 177.96, "J5": 177.96, "J6": 359.82}


def fold(v: float, period: float = 360.0) -> float:
    """折进 (-period/2, period/2]。**纯算术, 不认识任何限位。**

    折完是否合法由 within_limits 判, 这里不替它兜底。比如 J4 折 360 可能折出 -179.5,
    那正好落进 ±180 的 4° 死区里 —— 越限是真的, 但 fold 本身没做错, 不该私自把它挪走。
    """
    half = period / 2.0
    while v > half:
        v -= period
    while v <= -half:
        v += period
    return v


def within_limits(pose: dict) -> bool:
    """每个关节是否落在真机限位内。越限就是越限, 不修不猜。"""
    return all(abs(v) <= LIMITS[j] + 1e-9 for j, v in pose.items() if j in LIMITS)


def wrist_headroom_pose(pose: dict) -> dict:
    """位姿换成 ZYZ 等价解: 物理位姿一模一样, 但腕部有余量。

    实测 (2026-10-04) 录下来的位姿 J4 ≈ -163.8~-172.5°, 离真机 ±177.96° 限位只剩 ~9°。
    腕部没有余量 = "再转一点就出限位", 于是任何需要改变夹爪朝向的抓取候选都会被 IK 逼到
    ZYZ 翻转分支上; 而翻转分支要 J4 从 -169° 走到 +172°, 只能穿过 ±180° 的限位死区 ——
    走不过去, 孪生的 BRANCH_JUMP 保护因此**正确**地把候选判成不可用 (twin.py:530)。

    实测同一个橙子, 同一个位姿: 原始 grasp1 (J4=-169°) 7 个候选只有 2 个能生成轨迹;
    换成翻转解 (J4=+11°) 10 个里 9 个能。抓取失败的大头就在这。

    翻转是 (J4+180, -J5, J6+180), 末端位姿不变 —— 实测位置差 0.04mm / 姿态差 0.006°,
    相机是 link_6 的固定子节点 (mount_camera.py: parent_frame='link_6'), 所以画面也不变。

    **必须整表一起翻, 不能只翻一部分。** 这个翻转是对合, 且是腕关节空间上的等距变换:
    两个位姿一起翻, J4 与 J6 的差不变、J5 的差只变号, 所以**两两相对距离逐个保持不变**。
    2026-10-04 只翻了 grasp* 而把 home 留在原支路, 结果每次从 home 到观察位姿要在腕上
    转 179° (实测 home J4=-170.5, 翻后的 grasp1 J4=+11) —— 现场就是"莫名其妙的翻腕"。

    三个关节的规范化各不相同, 判据是**折的周期必须让结果落在该关节的限位之内**:
      - J4 限位 ±177.96 比整圈**窄** 4°, 折 360 的结果域 (-180,180] 超出限位, 所以
        除了折还要落进 [-90,90] 的余量带;
      - J5 取负即可, 合法值的相反数仍合法;
      - J6 限位 ±359.82 比整圈**宽**, 结果域 (-180,180] 含于限位, 折 360 就够。

    早退条件 abs(J4) <= 90 保证**幂等**: 翻转把 (90,180] → [-90,0)、[-180,-90) → (0,90],
    落点必在 [-90,90], 再翻一次不动。
    """
    out = dict(pose)
    if abs(out["J4"]) <= 90.0:
        return out
    out["J4"] = fold(out["J4"] + 180.0, 360.0)
    out["J5"] = -out["J5"]
    out["J6"] = fold(out["J6"] + 180.0, 360.0)
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
        # 整份位姿表统一换到腕部有余量的 ZYZ 等价解。见 wrist_headroom_pose:
        # 录下来的 J4 贴着 ±178° 限位, 不改的话任何改朝向的抓取候选都会被孪生判成不可达。
        #
        # **必须整表一起翻**: 翻转是等距变换, 全表一起翻则两两相对距离逐个不变 (一秒都不
        # 多走), 而只翻一部分会在两拨位姿之间造出 179° 的腕部大回转。所以这里不挑名字,
        # 凡是 J1..J6 的位姿都过一遍 —— default_traj_js 里的和顶层的 (递送/扔垃圾/放桌面)。
        for _key, _pose in list(self.default_traj_js.items()):
            if isinstance(_pose, dict) and "J4" in _pose:
                self.default_traj_js[_key] = wrist_headroom_pose(_pose)
        for _key, _pose in list(self.robot_config.items()):
            if isinstance(_pose, dict) and "J4" in _pose:
                self.robot_config[_key] = wrist_headroom_pose(_pose)
        # 翻完不该有越限的。J4 落 [-90,90]、J5 取负保号、J6 落 (-180,180] ⊂ ±359.82,
        # 三条都能证; 这里兜的是"以后有人往配置里塞了个越限位姿"。
        for _key, _pose in list(self.default_traj_js.items()):
            if isinstance(_pose, dict) and "J4" in _pose:
                assert within_limits(_pose), f"{_key} 翻转后仍越限: {_pose}"
        for _key, _pose in list(self.robot_config.items()):
            if isinstance(_pose, dict) and "J4" in _pose:
                assert within_limits(_pose), f"{_key} 翻转后仍越限: {_pose}"
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

    # ------------------------------------------------------------------
    # 抓取管线 (core/grasp_pipeline.py) 的配置接口 —— 原项目同款 API, 单臂化。
    # ------------------------------------------------------------------
    def get_arm_config(self, side: str = "left") -> dict:
        """Return the arm config dict (poses, twin_struct, ...). 单臂: side 被忽略。"""
        return dict(self.robot_config, twin_struct="rm65")

    def get_grasp_scoring(self, side: str = "left") -> dict:
        """Return grasp scoring / verification parameters (robot_config 的 grasp_scoring 段)."""
        return self.robot_config.get("grasp_scoring", {})

    def get_camera_intrinsics(self, side: str = "right") -> dict:
        """Return the wrist camera intrinsics {fx, fy, cx, cy}."""
        from core.transforms import _get_intrinsics
        fx, fy, cx, cy = _get_intrinsics(side)
        return {"fx": fx, "fy": fy, "cx": cx, "cy": cy}

    @property
    def shared(self) -> dict:
        """Return the shared configuration section (空段安全 —— 管线里都有默认值)."""
        return self.robot_config.get("shared", {})

    @property
    def sim_mode(self) -> bool:
        """本平台没有 sim 后端, 恒 False。"""
        return False
