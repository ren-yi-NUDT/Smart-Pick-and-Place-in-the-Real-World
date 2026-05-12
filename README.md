# Smart Pick and Place in the Real World

基于虚实结合双重推理架构的跨机器人智能抓取放置系统。采用 **Skill-DB + 硬件抽象层** 架构，Skill 代码与具体硬件解耦——切换机器人只需更换 Backend，Skill 逻辑零改动。

通过统一 CLI 调用封装好的机器人技能，集成 YOLO-World 开放词汇检测、AnyGrasp 抓取姿态生成、PyBullet 仿真轨迹规划，实现真实环境下的智能抓取与放置。

---

## 系统架构

```
┌──────────────────────────────────────────────────────────────────────┐
│                         run_skill.py (统一入口)                        │
│              argparse + JSON stdin → Skill Registry → skill.run()    │
└──────────────────────────────┬───────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                           skills/ (Skill-DB)                          │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────┐  ┌────────────┐ │
│  │pick_and_place│  │fetch_from    │  │look_around │  │capture_at  │ │
│  │  (高级skill)  │  │  _user       │  │            │  │ _handover  │ │
│  └──────┬───────┘  └──────────────┘  └────────────┘  └────────────┘ │
│         │  独立CLI调用（高级skill已内联等效逻辑）                        │
│  ┌──────┴──────────────────────────────────────────────┐             │
│  │  grasp │ place │ handover │ trash │ desk_place       │ (独立skill)│
│  └───────┴───────┴─────────┴───────┴──────────────────┘             │
│  ┌──────────────┐                                                     │
│  │pose_execute  │  (位姿/动作序列执行)                                  │
│  └──────────────┘                                                     │
└──────────────────────────────┬───────────────────────────────────────┘
                               │ self.arm / self.hand / self.camera ...
                               │ (懒加载 property)
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    core/abc.py (硬件抽象接口)                           │
│  BaseConfig │ BaseArm │ BaseHand │ BaseCamera │ BaseTwinClient        │
│  BaseTransforms │ BasePerception │ BaseVLM                             │
└──────────────────────────────┬───────────────────────────────────────┘
                               │
                     ROBOT_BACKEND 环境变量
                               │
              ┌────────────────┼────────────────┐
              ▼                ▼                ▼
┌───────────────────┐ ┌───────────────────┐ ┌──────────────┐
│  RM65 Backend     │ │  Igrape Backend   │ │  更多Backend  │
│  (默认)            │ │                   │ │  (可扩展)     │
│                   │ │                   │ │              │
│  Socket TCP       │ │  ROS2 Topics      │ │              │
│  + ROS1 TF       │ │  + WebSocket      │ │              │
│  + pyrealsense2  │ │  + PyBullet IK    │ │              │
│                   │ │  + ROS2 TF2       │ │              │
└───────────────────┘ └───────────────────┘ └──────────────┘
```

**关键设计：**
- `skills/base.py` 通过工厂函数创建硬件客户端，不直接 import 具体实现
- `ROBOT_BACKEND` 环境变量选择后端：`rm65`（默认）或 `igrape`
- 新增机器人只需在 `core/backends/` 下实现 ABC，无需修改任何 Skill 代码

---

## Backend 详解

### RM65 Backend（默认）

桌面级单臂系统，通过 TCP Socket 与 ROS 节点通信。

| 模块 | 通信方式 | 端口/地址 |
|------|----------|-----------|
| ArmClient | TCP Socket (4字节长度头 + JSON) | 127.0.0.1:8010 |
| HandClient | TCP Socket (纯 JSON) | 127.0.0.1:8000 |
| TwinClient | TCP Socket (纯 JSON 发, 4字节长度头 收) | 127.0.0.1:8020 |
| RealSenseCapture | pyrealsense2 本地采集 | USB |
| TransformationUtil | ROS1 TF (rospy) | ROS Master |

硬件：RM75-B 7-DOF 机械臂 + Inspire 灵巧手 + RealSense D455 相机

### Igrape Backend

人形双臂机器人，通过 ROS2 Topic + WebSocket 通信。

| 模块 | 通信方式 | Topic/地址 |
|------|----------|------------|
| IgrapeArm | ROS2 Publisher + Subscriber | `/arm/cmd_pos` (CmdSetMotorPosition) |
| IgrapeHand | ROS2 Publisher + Subscriber | `/inspire_hand/ctrl/right_hand` (JointState) |
| IgrapeTwinClient | PyBullet IK (本地) | SmartGraspPlanner |
| IgrapeCamera | WebSocket (异步) | `ws://192.168.3.16:8765` |
| IgrapeTransforms | ROS2 TF2 | `tf2_ros.Buffer` |

硬件：Igrape-bot3 人形机器人 + Inspire 灵巧手 + 远程 RGB-D 相机

**Igrape 特有配置** (`igrape_config.json`)：
- 位姿名映射：Skill-DB 名 → Igrape `actions.json` 名（如 `grasp1 → new_camera_2`）
- 帧名映射：`base_link → base`, `cam_link_grasp → camera_link`
- 关节映射：`J1-J7 (度)` ↔ `motor ID 21-27 (弧度)`

---

## 快速开始

### RM65（默认后端）

```bash
conda activate anygrasp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib

# 启动硬件服务
bash start1.bash   # ROS (灵巧手 :8000, 机械臂 :8010)
bash start2.bash   # Twin IK 服务 (:8020)

# 调用技能（默认 ROBOT_BACKEND=rm65）
echo '{"object":"orange","container":"green bowl"}' | python run_skill.py pick_and_place
```

### Igrape 后端

```bash
# 1. Source Igrape ROS2 workspace
source /home/zz/Code/IgrapeRobot3/IgrapeRobot3-task_planner_v3.0/install/setup.bash

# 2. 确保 Igrape 硬件服务已启动（ROS2 nodes + camera WebSocket）

# 3. 切换后端并调用技能
ROBOT_BACKEND=igrape python run_skill.py list
ROBOT_BACKEND=igrape echo '{"object":"orange","container":"green bowl"}' | python run_skill.py pick_and_place
```

---

## 项目结构

```
Smart-Pick-and-Place-in-the-Real-World/
├── run_skill.py                  # 统一 CLI 入口
├── robot_config.json             # RM65 机器人配置
├── igrape_config.json            # Igrape 后端配置（位姿映射、帧名、服务地址）
├── recorded_poses.json           # 录制的位姿库
│
├── core/                         # 硬件抽象层 + RM65 实现
│   ├── abc.py                    # 抽象接口 (BaseConfig, BaseArm, BaseHand, ...)
│   ├── config.py                 # RM65 Config (robot_config.json)
│   ├── arm.py                    # RM65 ArmClient (TCP :8010)
│   ├── hand.py                   # RM65 HandClient (TCP :8000)
│   ├── camera.py                 # RealSenseCapture (pyrealsense2)
│   ├── twin_client.py            # RM65 TwinClient (TCP :8020)
│   ├── transforms.py             # TransformationUtil (ROS1 TF) + 纯函数
│   ├── perception.py             # YOLO-World + AnyGrasp
│   ├── vlm.py                    # GLM-4.5V
│   ├── json_input.py             # JSON stdin 解析
│   │
│   └── backends/                 # 后端工厂 + Igrape 实现
│       ├── __init__.py           # 工厂函数 (ROBOT_BACKEND 环境变量)
│       └── igrape/
│           ├── __init__.py
│           ├── _ros2_context.py  # ROS2 单例生命周期 (rclpy init + spin 线程)
│           ├── _joint_map.py     # J1-J7(度) ↔ motor 21-27(弧度) 转换
│           ├── config.py         # IgrapeConfig (actions.json + 位姿映射)
│           ├── arm.py            # IgrapeArm (ROS2 /arm/cmd_pos + 阻塞等待)
│           ├── hand.py           # IgrapeHand (ROS2 /inspire_hand/ctrl/right_hand)
│           ├── camera.py         # IgrapeCamera (WebSocket RGB-D)
│           ├── twin_client.py    # IgrapeTwinClient (PyBullet SmartGraspPlanner)
│           ├── transforms.py     # IgrapeTransforms (ROS2 TF2)
│           ├── perception.py     # 沿用 RM65 Perception
│           └── vlm.py            # 沿用 RM65 VLMClient
│
├── skills/                       # Skill-DB
│   ├── base.py                   # Skill 基类 + 注册 + 工厂懒加载
│   ├── __init__.py               # 导入所有 skill 触发注册
│   ├── pick_and_place.py         # 高级：检测→抓取→放置
│   ├── fetch_from_user.py        # 高级：接收→放置
│   ├── look_around.py            # 高级：场景扫描 + VLM
│   ├── capture_at_handover.py    # 高级：handover 拍照
│   ├── pose_execute.py           # 位姿/动作序列执行
│   ├── grasp.py                  # 原子：视觉抓取
│   ├── place.py                  # 原子：视觉放置
│   ├── handover.py               # 原子：递交给用户
│   ├── trash.py                  # 原子：扔垃圾
│   └── desk_place.py             # 原子：放桌面
│
├── tools/                        # 开发工具
│   ├── pose_record.py            # 位姿录制
│   └── get_current_pose.py       # 读取当前关节角度
│
└── dependence/                   # 第三方依赖
    ├── twin_inference/           # 数字孪生推理 (PyBullet server)
    ├── anygrasp_sdk/             # AnyGrasp 抓取检测 SDK
    ├── yolo_world/               # YOLO-World 模型
    └── smart_pick_and_place_ws/  # ROS catkin 工作空间
```

---

## 添加新 Backend

以添加新机器人 `my_robot` 为例：

#### 1. 创建 Backend 目录

```
core/backends/my_robot/
├── __init__.py
├── config.py         # 继承 BaseConfig
├── arm.py            # 继承 BaseArm
├── hand.py           # 继承 BaseHand
├── camera.py         # 继承 BaseCamera
├── twin_client.py    # 继承 BaseTwinClient
├── transforms.py     # 继承 BaseTransforms
├── perception.py     # 继承 BasePerception
└── vlm.py            # 继承 BaseVLM
```

#### 2. 实现抽象接口

```python
# core/backends/my_robot/arm.py
from core.abc import BaseArm

class MyRobotArm(BaseArm):
    def connect(self) -> bool:
        # 连接你的机器人
        ...
    def move_to_named_pose(self, pose_dict: dict, speed: int = 30) -> bool:
        # pose_dict = {"J1": deg, ..., "J7": deg}
        # 内部转换为你的协议
        ...
```

#### 3. 注册到工厂

在 `core/backends/__init__.py` 的每个 `create_xxx` 函数中添加分支：

```python
def create_arm(**kwargs):
    if BACKEND == "igrape":
        ...
    elif BACKEND == "my_robot":
        from core.backends.my_robot.arm import MyRobotArm
        return MyRobotArm(**kwargs)
    ...
```

#### 4. 使用

```bash
ROBOT_BACKEND=my_robot python run_skill.py pick_and_place
```

---

## Skill 一览

### 高级 Skill（组合流程）

| Skill | 说明 | 输入 |
|-------|------|------|
| `pick_and_place` | 检测→抓取→放置完整流程 | `object` + `container` |
| `fetch_from_user` | 从用户手中接收→放置 | `container` |
| `look_around` | 移动到观测位姿拍照，VLM 分析场景 | 无 |
| `capture_at_handover` | 移动到 handover 位拍照，VLM 识别物品 | 无 |
| `pose_execute` | 执行位姿/动作序列（支持手势） | `sequence` 或 `command` |

### container 参数

| container 值 | 模式 | 行为 |
|---|---|---|
| 容器名称 (如 `"green bowl"`) | 桌面放置 | YOLO 检测容器位置，生成放置轨迹 |
| `"person"` | 人机递物 | 平滑轨迹到 handover 位姿，松手 |
| `"trash"` | 扔垃圾 | 移动到垃圾桶位姿，松手 |
| `"desk"` | 放桌面 | 随机选择3个预设位姿之一 |

---

## 通信协议 (RM65 Backend)

| 端口 | 服务 | 发送协议 | 接收协议 |
|------|------|----------|----------|
| 8000 | 灵巧手 | 纯 JSON | 纯 JSON |
| 8010 | 机械臂 | 4字节大端长度头 + JSON | 纯 JSON |
| 8020 | Twin IK | 纯 JSON | 4字节大端长度头 + JSON |

**注意：** `robot_config.json` 和机械臂命令使用角度制（度）。Twin 服务返回的轨迹使用弧度制，Skill 内部会做 `traj / np.pi * 180` 转换。

## 通信协议 (Igrape Backend)

| Topic / 地址 | 服务 | 消息类型 |
|---|---|---|
| `/arm/cmd_pos` | 右臂控制 | `bodyctrl_msgs/CmdSetMotorPosition` |
| `/inspire_hand/ctrl/right_hand` | 灵巧手控制 | `sensor_msgs/JointState` |
| `/joint_states` | 关节状态反馈 | `sensor_msgs/JointState` |
| `ws://192.168.3.16:8765` | RGB-D 相机 | WebSocket (JPEG + PNG 16-bit) |
| `ws://192.168.3.11:8775` | YOLO+AnyGrasp 抓取服务 | WebSocket (base64 JSON) |

**关节映射：** `J1-J7 (度, 0-360)` ↔ `motor ID 21-27 (弧度, ±π)`

**灵巧手：** Skill-DB `0-1000` → Igrape `0.0-1.0`（线性映射）

**部署架构：**
- x86 主机：Agent + Body + Chassis + SLAM
- Orin (192.168.3.16)：RGB-D 相机服务 + TTS
- 服务器 (192.168.3.11)：YOLO + SAM + AnyGrasp

---

## 开发原则

- **流水线执行中禁止跨 Skill 实例化**：高级 skill 需执行子任务时，使用 `self.control_arm` / `self.control_hand` 内联逻辑，而非创建新实例
- **Skill 的 `run()` 必须先检查 `kwargs` 再回退 stdin**
- **位姿查询统一用 `Config.get_pose()`**：不直接访问 `config.robot_config` 或 `config.default_traj_js`
- **硬件客户端必须通过工厂创建**：不在 skill 中直接 `from core.arm import ArmClient`

---

## 依赖

| 依赖 | 用途 |
|------|------|
| ROS Noetic / ROS2 | 机器人控制和坐标变换 |
| PyBullet | 物理仿真和逆运动学 |
| YOLO-World (Ultralytics) | 开放词汇目标检测 |
| AnyGrasp SDK | 抓取姿态生成 |
| pyrealsense2 | RealSense D455 驱动 (RM65) |
| CUDA / cuDNN | GPU 加速 |
| scipy, numpy, open3d, PIL, termcolor | 数学/可视化 |

## License

See [LICENSE](LICENSE).
