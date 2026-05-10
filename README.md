# Smart Pick and Place in the Real World

基于虚实结合双重推理架构的桌面级智能机械臂 Pick-and-Place 系统。采用 **Skill-DB 架构**，通过统一 CLI 调用封装好的机器人技能，集成 YOLO-World 开放词汇检测、AnyGrasp 抓取姿态生成、PyBullet 仿真轨迹规划，实现真实环境下的智能抓取与放置。

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                       run_skill.py (统一入口)                      │
│            argparse + JSON stdin → Skill Registry → skill.run()  │
└──────────┬───────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────┐
│                         skills/ (Skill-DB)                        │
│  ┌─────────────┐  ┌─────────────┐  ┌───────────┐  ┌───────────┐ │
│  │pick_and_place│  │fetch_from   │  │look_around│  │capture_at │ │
│  │  (高级skill) │  │  _user      │  │           │  │ _handover │ │
│  └──────┬──────┘  └─────────────┘  └───────────┘  └───────────┘ │
│         │ 独立CLI调用（高级skill已内联等效逻辑）                     │
│  ┌──────┴──────────────────────────────────────────┐             │
│  │  grasp │ place │ handover │ trash │ desk_place  │ (独立skill) │
│  └───────┴───────┴─────────┴───────┴──────────────┘             │
│  ┌─────────────┐                                                  │
│  │pose_execute │  (位姿/动作序列执行)                               │
│  └─────────────┘                                                  │
└──────────┬───────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────┐
│                  core/ (基础设施 + 硬件抽象层)                      │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐     │
│  │  core/drivers/  (硬件抽象)                                │     │
│  │  ArmDriver (ABC)  ── RM75ArmDriver  (RM-75 7DOF)        │     │
│  │  HandDriver (ABC) ── InspireHandDriver (Inspire 灵巧手)  │     │
│  │  factory.py ← robot_profile.json → 具体驱动实例           │     │
│  └──────────────────────────┬──────────────────────────────┘     │
│                             │                                    │
│  config │ camera │ twin_client │ transforms                      │
│  perception │ vlm │ json_input                                    │
└──────────┬──────────────────┬────────────────────────────────────┘
           │                  │
     Socket Clients     Socket Servers
     (8010/8000/8020)    (ROS Nodes + Twin Server)
```

**三进程分布式架构：**
- **进程1 (start1.bash)**: ROS系统启动 — 机械臂驱动、相机节点、灵巧手节点
- **进程2 (start2.bash)**: 数字孪生推理服务器 — PyBullet物理仿真，IK求解与碰撞检测
- **主进程 (run_skill.py)**: 通过 CLI 调用技能

### 硬件抽象层

系统通过 **Driver 抽象层** 实现硬件解耦，使 Skill 层代码与具体机械臂/灵巧手型号无关：

```
robot_profile.json          ← 硬件参数配置（关节名、DOF、端口、URDF 路径等）
    │
    ▼
core/config.py              ← 加载 profile，提供关节无关辅助方法
    │                          pose_to_list() / get_default_js_rad() / arm_struct_name
    ▼
core/drivers/               ← 抽象接口 + 具体实现
  ├── arm_driver.py          ArmDriver ABC (connect / move_to_named_pose / execute_trajectory)
  ├── hand_driver.py         HandDriver ABC (connect / open / close / is_grasping)
  ├── rm75_driver.py         RM-75 7-DOF 机械臂实现
  ├── inspire_driver.py      Inspire 灵巧手实现
  └── factory.py             create_arm_driver() / create_hand_driver()
    │
    ▼
skills/base.py              ← 懒加载属性通过工厂创建 driver 实例
```

**设计要点：**

1. **Profile 驱动**：`robot_profile.json` 声明硬件型号（`driver: "rm75"`）和参数（关节名、端口、URDF 路径），Config 加载后提供统一的关节无关接口
2. **工厂模式**：`factory.py` 根据配置中的 `driver` 字段选择对应的 Driver 实现，Skill 层无需知道具体型号
3. **零配置兼容**：`robot_profile.json` 不存在时，Config 自动从旧常量合成 RM-75 默认 profile，现有部署无需任何改动
4. **关节无关**：Skill 中不再出现 `J1`-`J7` 或 `range(1, 8)` 等硬编码，统一使用 `config.pose_to_list()` 和 `config.get_default_js_rad()` 转换
5. **孪生服务解耦**：Twin 服务器从 profile 读取 URDF 路径，struct 验证改为动态检查，不再硬编码机械臂型号

### 跨臂适配流程

适配新机械臂只需 4 步，**无需修改任何 Skill 代码**：

1. **创建 Profile**：复制 `robot_profile.json`，修改关节名、DOF、端口、URDF 路径
2. **实现 Driver**：在 `core/drivers/` 中新建实现类（如 `franka_driver.py`），实现 `ArmDriver` ABC
3. **注册 Driver**：在 `factory.py` 的 `ARM_DRIVERS` 字典中注册
4. **提供位姿和 URDF**：在 `robot_config.json` 中录制新臂的位姿，为新臂提供 PyBullet URDF

## 快速开始

### 前置条件

- ROS Noetic
- Conda 环境 `anygrasp` (Python 3.9)
- Intel RealSense D455 相机
- RM75-B 7-DOF 机械臂 + Inspire 灵巧手
- CUDA / cuDNN

### 启动流程 A：面向 OpenClaw（AI Agent 模式）

通过 OpenClaw 启动 AI Agent（CMLLR），以自然语言对话方式操控机械臂。适用于演示和交互场景。

**前置：** OpenClaw 已安装（`openclaw --version`），`openclaw-configs` 分支已 checkout 到工作目录。

```bash
# ── 终端 1：ROS 硬件服务 ──
bash start1.bash    # 灵巧手 :8000, 机械臂 :8010

# ── 终端 2：数字孪生 IK 服务 ──
bash start2.bash    # Twin IK :8020

# ── 终端 3：启动 OpenClaw Agent ──
openclaw             # 加载 workspace/ (IDENTITY, SOUL, skills/) 启动 CMLLR
```

启动后 CMLLR 会自动加载 `SOUL.md` → `USER.md` → `skills/` → `MEMORY.md`，之后可通过自然语言对话操控机器人：

```
> 把橘子放到绿色碗里
> 递给我那个瓶子
> 看看桌上有什么
> 做个挥手的动作
```

Agent 在后台调用 `run_skill.py` 执行具体技能，无需手动输入 JSON。

### 启动流程 B：面向测试（CLI 直调模式）

直接通过命令行调用技能，适用于开发调试和自动化测试。

```bash
# 0. 激活环境
conda activate anygrasp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib

# 1. 确保硬件连接
ping 192.168.1.19     # 机械臂
ping 192.168.11.209   # 灵巧手

# 2. 启动硬件服务（在 2 个终端中分别启动）
bash start1.bash   # ROS (灵巧手 8000, 机械臂 8010)
bash start2.bash   # Twin IK 服务 (8020)

# 3. 调用技能
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World

# 查看所有可用 skill
python run_skill.py list

# 抓取物品放到容器里
echo '{"object":"orange","container":"green bowl"}' | python run_skill.py pick_and_place

# 把物品递给用户
echo '{"object":"bottle","container":"person"}' | python run_skill.py pick_and_place

# 扔垃圾
echo '{"object":"wrapper","container":"trash"}' | python run_skill.py pick_and_place

# 放到桌面
echo '{"object":"cup","container":"desk"}' | python run_skill.py pick_and_place

# 从用户手中接收物品
echo '{"container":"pink plate"}' | python run_skill.py fetch_from_user

# 环顾桌面拍照
python run_skill.py look_around

# 执行动作序列
echo '{"sequence":[{"arm":"home","hand":"open","delay":0.5}]}' | python run_skill.py pose_execute
```

**快速验证服务是否就绪：**
```bash
lsof -ti:8000,8010,8020  # 应返回 3 个 PID
```

## Skill 一览

### 高级 Skill（组合流程）

| Skill | 说明 | 输入 |
|-------|------|------|
| `pick_and_place` | 检测→抓取→放置完整流程 | `object` + `container` |
| `fetch_from_user` | 从用户手中接收→放置 | `container` |
| `look_around` | 移动到观测位姿拍照，VLM 分析场景 | 无 |
| `capture_at_handover` | 移动到 handover 位拍照，VLM 识别物品 | 无 |
| `pose_execute` | 执行位姿/动作序列（支持手势） | `sequence` 或 `command` |

### 原子 Skill（单步操作，仅用于独立 CLI 调用）

> **注意**：高级 skill（pick_and_place、fetch_from_user）内部已内联等效逻辑，不再实例化调用这些原子 skill，以避免创建额外的 TCP 连接。原子 skill 仅保留给独立 CLI 调用。

| Skill | 说明 |
|-------|------|
| `grasp` | 视觉抓取（YOLO 检测 + AnyGrasp + Twin 轨迹） |
| `place` | 视觉放置（检测容器位置 → 生成放置轨迹） |
| `handover` | 递交给人（插值轨迹经中间点到 handover 位姿） |
| `trash` | 扔垃圾（移动到垃圾桶位姿松手） |
| `desk_place` | 放桌面（随机选择预设位姿） |

### container 参数

| container 值 | 模式 | 行为 |
|---|---|---|
| 容器名称 (如 `"green bowl"`) | 桌面放置 | YOLO 检测容器位置，生成放置轨迹 |
| `"person"` | 人机递物 | 平滑轨迹到 handover 位姿，松手 |
| `"trash"` | 扔垃圾 | 移动到垃圾桶位姿，松手 |
| `"desk"` | 放桌面 | 随机选择3个预设位姿之一 |

## 配置系统

系统使用两个互补的配置文件：

### `robot_profile.json` — 硬件描述

声明机械臂和灵巧手的型号、通信参数和运动学特征。切换硬件时只需更换此文件。

```json
{
    "arm": {
        "driver": "rm75",                    // driver 注册名
        "service_name": "/right_arm/movement_control",
        "struct_name": "left_arm",           // 孪生服务中的结构体名
        "num_joints": 7,
        "joint_names": ["J1","J2","J3","J4","J5","J6","J7"],
        "host": "127.0.0.1", "port": 8010
    },
    "hand": {
        "driver": "inspire",
        "service_name": "/left_hand/movement_control",
        "gestures": {"close": [0,0,0,460,0,0], "open": [1000,1000,1000,1000,1000,0]},
        "host": "127.0.0.1", "port": 8000
    },
    "twin": {
        "host": "127.0.0.1", "port": 8020,
        "urdf_path": "dependence/.../easy_single_arm_bullet.urdf",
        "robot_config_path": "dependence/.../robot_config.json"
    },
    "frames": {
        "base_link": "base_link",
        "camera_link": "cam_link_grasp",
        "hand_effector": "L_hand_endeffector",
        "arm_end_link": "Link7"
    }
}
```

### `robot_config.json` — 任务位姿

存储具体任务中的关节位姿（抓取观测位、放置位、handover 位等），换臂后需要重新录制。

| 存储位置 | 包含的位姿 | 查询方式 |
|----------|-----------|----------|
| `default_traj_js` 字段内 | `grasp1-4`、`place1-2` | `config.default_traj_js[name]` |
| JSON 顶层 | `handover_pose`、`get_ready_to_handover_*`、`throw_to_trash_pose`、`desk_pose_*` | `config.robot_config.get(name)` |

**统一查询方式**：始终使用 `config.get_pose(name)` 方法，该方法会先查顶层再查 `default_traj_js`。

### Config 关节无关接口

Skill 层通过 Config 提供的辅助方法访问位姿数据，不直接操作关节名：

| 方法 | 说明 |
|------|------|
| `config.pose_to_list(pose_dict)` | 位姿 dict → 有序值列表（按 profile 中的 joint_names 排序） |
| `config.get_default_js_rad(name)` | 获取指定位姿的弧度值列表（度→弧度） |
| `config.arm_struct_name` | 孪生服务的结构体名（如 `"left_arm"`） |
| `config.arm_num_joints` | 关节数量 |
| `config.arm_joint_names` | 关节名列表 |

## 项目结构

```
Smart-Pick-and-Place-in-the-Real-World/
├── run_skill.py              # 统一 CLI 入口
├── robot_config.json         # 任务位姿配置（关节角度、坐标系名称）
├── robot_profile.json        # 硬件描述配置（型号、端口、关节名、URDF 路径）
├── recorded_poses.json       # 录制的位姿库
│
├── skills/                   # Skill-DB
│   ├── base.py               # Skill 基类 + 注册机制 + 懒加载硬件
│   ├── __init__.py           # 导入所有 skill 触发注册
│   ├── pick_and_place.py     # 高级：抓取+放置
│   ├── fetch_from_user.py    # 高级：从用户接收
│   ├── look_around.py        # 高级：场景扫描
│   ├── capture_at_handover.py# 高级：handover 拍照
│   ├── pose_execute.py       # 位姿/动作序列执行
│   ├── grasp.py              # 原子：视觉抓取
│   ├── place.py              # 原子：视觉放置
│   ├── handover.py           # 原子：递交给用户
│   ├── trash.py              # 原子：扔垃圾
│   └── desk_place.py         # 原子：放桌面
│
├── core/                     # 共享基础设施
│   ├── config.py             # 集中配置管理 + profile 加载 + 关节无关辅助方法
│   ├── drivers/              # 硬件抽象层
│   │   ├── arm_driver.py     # ArmDriver ABC
│   │   ├── hand_driver.py    # HandDriver ABC
│   │   ├── rm75_driver.py    # RM-75 机械臂实现
│   │   ├── inspire_driver.py # Inspire 灵巧手实现
│   │   └── factory.py        # Driver 工厂（根据 profile 创建实例）
│   ├── arm.py                # RM-75 Socket 客户端 (底层, :8010)
│   ├── hand.py               # Inspire Socket 客户端 (底层, :8000)
│   ├── camera.py             # RealSense RGB-D 采集
│   ├── twin_client.py        # 数字孪生客户端 (:8020)
│   ├── transforms.py         # ROS TF 坐标变换
│   ├── perception.py         # YOLO-World + AnyGrasp 封装
│   ├── vlm.py                # GLM-4.5V 视觉语言模型客户端
│   └── json_input.py         # JSON stdin 解析
│
├── tools/                    # 开发工具（非 skill）
│   ├── pose_record.py        # 位姿录制（直连机械臂 SDK）
│   └── get_current_pose.py   # 读取当前关节角度
│
├── dependence/               # 第三方依赖
│   ├── twin_inference/       # 数字孪生推理（独立进程）
│   ├── anygrasp_sdk/         # AnyGrasp 抓取检测 SDK
│   ├── yolo_world/           # YOLO-World 模型
│   └── smart_pick_and_place_ws/ # ROS catkin 工作空间
│
├── start1.bash               # 启动 ROS 服务
├── start2.bash               # 启动 Twin IK 服务
└── start.bash                # 一键启动全部
```

## Skill 基类

所有 skill 继承 `skills.base.Skill`，通过 `@register_skill("name")` 注册：

```python
from skills.base import Skill, register_skill

@register_skill("my_skill")
class MySkill(Skill):
    def run(self, **kwargs):
        # 通过 self.arm, self.hand, self.camera 等懒加载属性访问硬件
        pass
```

硬件资源通过 property 懒加载，首次访问时由 `factory.py` 根据 `robot_profile.json` 创建对应的 Driver 实例。Skill 层看到的 `self.arm` 是 `ArmDriver` 抽象接口，不依赖具体型号。

## 添加新 Skill

1. 在 `skills/` 下创建 `my_skill.py`
2. 继承 `Skill`，添加 `@register_skill("my_skill")`
3. 实现 `run(self, **kwargs)`，使用 `config.get_pose()` / `config.pose_to_list()` 访问位姿
4. 完成。调用：`echo '{"key":"value"}' | python run_skill.py my_skill`

## 添加新机械臂

1. 在 `core/drivers/` 中创建 `my_arm_driver.py`，实现 `ArmDriver` ABC
2. 在 `factory.py` 中注册：`ARM_DRIVERS["my_arm"] = MyArmDriver`
3. 创建 `robot_profile.json`，设置 `"driver": "my_arm"` 和对应的关节参数
4. 提供新臂的 URDF（PyBullet 仿真用），在 `robot_config.json` 中录制新臂的任务位姿
5. 如需新手/夹爪，同理实现 `HandDriver` 并注册

## 通信协议

| 端口 | 服务 | 发送协议 | 接收协议 |
|------|------|----------|----------|
| 8000 | 灵巧手 | 纯 JSON | 纯 JSON |
| 8010 | 机械臂 | 4字节大端长度头 + JSON | 纯 JSON |
| 8020 | Twin IK | 纯 JSON | 4字节大端长度头 + JSON |

**灵巧手指令：** `{"src": "/left_hand/movement_control", "type": "set", "cmd": [a0,a1,a2,a3,a4,a5]}`
- `[小指, 无名指, 中指, 食指, 拇指, 拇指外展]`，0=弯曲，1000=张开

**手势预设：** `open`, `close`, `peace`, `rock`, `pointing`, `thumbs_up`, `ok`, `grab`

**机械臂指令：** `{"srv": "/right_arm/movement_control", "cmd": [{"type":"start"},{"type":"js","act":{J1:...},"speed":20,"block":true},{"type":"end"}]}`

**注意：** `robot_config.json` 使用角度制，Twin 返回的轨迹使用弧度制，skill 内部会进行转换。

## 数字孪生服务

| 类型 | 说明 |
|------|------|
| `trajectory_generation` | 单目标线性轨迹（含碰撞检测） |
| `trajectory_generation2` | 多目标线性轨迹（含碰撞检测和 Z 轴安全检查） |
| `reachability_check` | 可达性检查 |
| `collision_check` | 碰撞检测 |
| `IK_calculation` | 逆运动学求解 |

## 开发工具

```bash
# 录制位姿（直连机械臂 SDK，不需要 ROS 服务）
python tools/pose_record.py record --name "home"

# 查看当前关节角度
python tools/get_current_pose.py
```

## 硬件

| 设备 | 型号 | 连接 |
|------|------|------|
| 机械臂 | RM75-B (7DOF) | 192.168.1.19:8010 |
| 灵巧手 | Inspire Hand | 192.168.11.209:8000 (Modbus) |
| 相机 | RealSense D455 | USB, 640x480@30fps |

## 开发原则

- **流水线执行中禁止跨 Skill 实例化**：高级 skill（如 `pick_and_place`、`fetch_from_user`）需要执行子任务（递送、扔垃圾、放桌面）时，必须使用 `self.control_arm` / `self.control_hand` 内联逻辑，而非 `new Skill()` 创建新实例。新实例会建立额外的 TCP 连接（arm 8010、hand 8000），引入延迟和连接冲突，破坏阶段间的无缝衔接。
- **Skill 的 `run()` 方法必须先检查 `kwargs`**：`run_skill.py` 从 stdin 读取 JSON 后以 `kwargs` 传入。Skill 应先检查 `kwargs`（`if kwargs.get("field"): data = kwargs`），仅在 `kwargs` 为空时才回退到 `self.json_parser.get_command()`，否则 stdin 已被消费，parser 读不到数据。
- **位姿操作必须使用 Config 辅助方法**：不直接访问关节名（`pose["J1"]`）或 `range(1,8)` 循环，使用 `config.pose_to_list()`、`config.get_default_js_rad()`、`config.arm_struct_name` 等关节无关接口，确保 Skill 代码可跨臂复用。

## 依赖

- **ROS Noetic** — 机器人控制和坐标变换
- **PyBullet** — 物理仿真和逆运动学
- **YOLO-World (Ultralytics)** — 开放词汇目标检测
- **AnyGrasp SDK** — 抓取姿态生成
- **pyrealsense2** — RealSense D455 驱动
- **CUDA / cuDNN** — GPU 加速
- scipy, numpy, open3d, PIL, termcolor

## License

See [LICENSE](LICENSE).
