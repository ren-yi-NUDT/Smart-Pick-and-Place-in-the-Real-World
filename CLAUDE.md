# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指导。

## 项目概述

本项目为大创项目"基于虚实结合双重推理架构的桌面级智能机械臂平台"的机器人抓取放置系统。采用 **Skill-DB 架构**，通过统一 CLI 入口 `run_skill.py` 调用封装好的机器人技能，集成 YOLO-World 开放词汇检测、AnyGrasp 抓取姿态生成、PyBullet 仿真轨迹规划，实现真实环境下的智能抓取与放置。

核心能力：
- **抓取放置 (Pick and Place)**：视觉驱动抓取物体并放置到容器中
- **递送 (Handover)**：将物体递送给用户或从用户手中接收物体
- **扔垃圾/放桌面**：使用预定义位姿将物品扔进垃圾桶或放到桌面上
- **环视 (Look Around)**：扫描工作空间，使用 GLM-4.5V VLM 分析场景
- **位姿录制 (Pose Recording)**：录制和回放机械臂位姿及动作序列

## 运行系统

系统需要通过 `start.bash` 启动 3 个并发进程：

```bash
./start.bash  # 在 3 个 gnome-terminal 中启动所有脚本
```

或单独运行：
1. **start1.bash**：ROS bringup — 构建工作空间并启动 ROS 节点（机器人驱动、相机、灵巧手）
2. **start2.bash**：孪生推理服务 — PyBullet 仿真用于逆运动学/轨迹生成（端口 8020）
3. **start3.bash**：主规划器 — 调用 `run_skill.py` 执行技能

**Conda 环境**：`anygrasp`（Python 3.9）

**cuDNN 库路径**（AnyGrasp 运行必需）：
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib
```

**硬件 IP 地址**：
- 机械臂：192.168.1.19
- 灵巧手：192.168.11.210

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
│  └─────────────┘  └─────────────┘  └───────────┘  └───────────┘ │
│  ┌──────────────────────────────────────────────────┐            │
│  │grasp│place│handover│trash│desk_place│pose_execute│ (独立skill)│
│  └────┴────┴───────┴────┴─────────┴───────────────┘            │
└──────────┬───────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────┐
│                          core/ (基础设施)                          │
│  config │ arm │ hand │ camera │ twin_client │ transforms          │
│  perception │ vlm │ json_input                                   │
└──────────┬──────────────────────┬────────────────────────────────┘
           │                      │
     Socket Clients         Socket Servers
     (8010/8000/8020)       (ROS Nodes + Twin Server)
```

## 调用方式

通过 `run_skill.py` 统一调用技能，JSON 从 stdin 传入：

```bash
conda activate anygrasp

# 查看所有可用 skill
python run_skill.py list

# 抓取物品放到容器里
echo '{"object":"orange","container":"green bowl"}' | python run_skill.py pick_and_place

# 递送物品给用户
echo '{"object":"bottle","container":"person"}' | python run_skill.py pick_and_place

# 扔垃圾
echo '{"object":"wrapper","container":"trash"}' | python run_skill.py pick_and_place

# 放到桌面
echo '{"object":"cup","container":"desk"}' | python run_skill.py pick_and_place

# 从用户手中接收物品
echo '{"container":"trash"}' | python run_skill.py fetch_from_user

# 环顾桌面拍照
python run_skill.py look_around
```

### JSON 输入格式

- `object`（必需）：要抓取的物体名称，支持逗号分隔的多个类别（OR 逻辑）
- `container`（必需）：放置目标容器，或特殊模式关键字
- `direction`（可选）：空间提示（尚未实现）

### 特殊容器模式
| 容器值 | 模式 | 行为 |
|---|---|---|
| `"person"` | 递送 | 经中间路径点平滑运动到 handover 位姿，张开手 |
| `"trash"`、`"垃圾桶"`、`"garbage"`、`"bin"` | 扔垃圾 | 运动到扔垃圾位姿，松手 |
| `"desk"`、`"桌子"`、`"table"` | 放桌面 | 从 3 个预定义桌面位姿中随机选择，松手 |

## 项目结构

```
Smart-Pick-and-Place-in-the-Real-World/
├── run_skill.py              # 统一 CLI 入口
├── robot_config.json         # 机器人配置（关节位姿、坐标系名称）
├── recorded_poses.json       # 录制的位姿库
│
├── skills/                   # Skill-DB
│   ├── base.py               # Skill 基类 + 注册机制 + 懒加载硬件
│   ├── __init__.py           # 导入所有 skill 触发注册
│   ├── pick_and_place.py     # 高级：抓取+放置（含 handover/trash/desk 内联）
│   ├── fetch_from_user.py    # 高级：从用户接收（含 trash/desk 内联）
│   ├── look_around.py        # 高级：场景扫描 + VLM 分析
│   ├── capture_at_handover.py# 高级：handover 拍照
│   ├── pose_execute.py       # 位姿/动作序列执行（支持手势）
│   ├── grasp.py              # 独立：视觉抓取
│   ├── place.py              # 独立：视觉放置
│   ├── handover.py           # 独立：递交给用户
│   ├── trash.py              # 独立：扔垃圾
│   └── desk_place.py         # 独立：放桌面
│
├── core/                     # 共享基础设施
│   ├── config.py             # 集中配置管理（含 get_pose 统一查询）
│   ├── arm.py                # 机械臂 Socket 客户端 (:8010)
│   ├── hand.py               # 灵巧手 Socket 客户端 (:8000)
│   ├── camera.py             # RealSense RGB-D 采集
│   ├── twin_client.py        # 数字孪生客户端 (:8020)
│   ├── transforms.py         # ROS TF 坐标变换 + 工具函数
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

## 关键文件

### Skills (`skills/`)
| 文件 | 用途 |
|---|---|
| `base.py` | Skill 基类、注册机制、懒加载硬件属性（arm/hand/camera/twin/perception/vlm） |
| `pick_and_place.py` | 主流水线：检测 → 抓取 → 放置（handover/trash/desk 逻辑已内联） |
| `fetch_from_user.py` | 反向流水线：移到 handover 位 → 接收物品 → 放置 |
| `look_around.py` | 循环 grasp1-4 拍照，VLM 分析场景 |
| `grasp.py` | 视觉抓取：YOLO 检测 + AnyGrasp + Twin 轨迹 |
| `pose_execute.py` | 位姿/动作序列执行，支持手势预设 |

### Core (`core/`)
| 文件 | 用途 |
|---|---|
| `config.py` | 配置加载，`get_pose()` 统一位姿查询（先查顶层再查 `default_traj_js`） |
| `arm.py` | 机械臂 TCP 客户端（端口 8010），4 字节大端长度前缀协议 |
| `hand.py` | 灵巧手 TCP 客户端（端口 8000） |
| `camera.py` | RealSense RGB-D 采集（640x480, 30fps） |
| `twin_client.py` | 孪生推理 TCP 客户端（端口 8020） |
| `transforms.py` | ROS TF 坐标变换、相机投影、3D 可视化工具函数 |
| `perception.py` | YOLO-World + AnyGrasp 封装 |
| `vlm.py` | GLM-4.5V 视觉语言模型 API 客户端 |

### 孪生推理系统 (`dependence/twin_inference/`)
| 文件 | 用途 |
|---|---|
| `twin.py` | PyBullet socket 服务（端口 8020），逆运动学求解和轨迹生成 |
| `robot.py` | 机器人模型：`ErdaijiRobot` 类，`Arm`/`Hand`/`Gripper`/`Head` 结构体 |
| `sim_world.py` | PyBullet 物理仿真环境 |
| `utils.py` | 变换矩阵、SLERP 插值、可视化辅助 |
| `p_utils.py` | PyBullet 关节/连杆/碰撞工具函数 |

### ROS 工作空间 (`dependence/smart_pick_and_place_ws/`)
| 文件 | 用途 |
|---|---|
| `src/rm_65_pkg/src/arm_75_bringup.py` | 机械臂 ROS bringup 节点 |
| `src/rm_65_pkg/src/mount_camera.py` | 相机安装/标定节点 |
| `src/rm_65_pkg/src/inspire_hand_bringup.py` | Inspire 灵巧手 ROS 节点 |
| `src/rm_65_pkg/src/hand_controller_modbus.py` | 通过 Modbus 协议控制灵巧手 |
| `src/rm_description/urdf/SingleArm/` | 仿真用 URDF 模型和机器人配置 |

## Socket 通信协议

| 端口 | 服务 | 协议 | 消息格式 |
|---|---|---|---|
| 8000 | 灵巧手控制 | TCP | JSON：`{"src": "/left_hand/movement_control", "type": "set"/"get", "cmd": [...]}` |
| 8010 | 机械臂控制 | TCP | 4 字节大端长度前缀 + JSON：`{"srv": "/right_arm/movement_control", "cmd": [{"type": "start"}, {"type": "js", "act": {...}, "speed": N, "block": bool}, {"type": "end"}]}` |
| 8020 | 孪生推理 | TCP | 请求：纯 JSON；响应：4 字节大端长度前缀 + JSON |

**重要**：`robot_config.json` 和机械臂命令中的关节角度使用**角度制（度）**。孪生推理返回的轨迹使用**弧度制**，skill 内部会进行转换（除以 π × 180）。

## 位姿配置说明

`robot_config.json` 中的预定义位姿分两处存储，应使用 `Config.get_pose()` 统一查询：

| 存储位置 | 包含的位姿 |
|----------|-----------|
| `default_traj_js` 字段内 | `grasp1-4`、`place1-2` |
| JSON 顶层 | `handover_pose`、`get_ready_to_handover_*`、`throw_to_trash_pose`、`desk_pose_*`、`look_over_what_in_user_hand_pose` |

## 孪生推理服务类型

| 类型 | 描述 | 响应 |
|---|---|---|
| `reachability_check` | 检查目标位姿是否可达 + 碰撞检测 | `is_reached`、`delta_xyz`、`delta_rpy`、`is_collided` |
| `collision_check` | 与可达性检查相同 | 同上 |
| `IK_calculation` | 与可达性检查相同 | 同上 |
| `trajectory_generation` | 单目标线性轨迹 + 碰撞检测 | `trajectory`（弧度）、`trajectory_ee`、`infos` |
| `trajectory_generation2` | 多目标线性轨迹 + Z 轴高度安全检查 | 同上 + `is_z_safe`、`unsafe_links` |

## 检测与抓取

- **物体检测**：YOLO-World（yolov8x-worldv2.pt）— 开放词汇，支持任意类别名称
- **抓取检测**：AnyGrasp SDK — 生成 top-50 抓取候选，使用 YOLO 检测边界框（20px 边距）过滤
- **多类别**：逗号分隔值使用 OR 逻辑（如 `"apple,orange,fruit"`）

## 灵巧手手势预设

6 值数组：`[小指, 无名指, 中指, 食指, 拇指, 拇指外展]`（0=弯曲，1000=伸直）

| 手势 | 数值 |
|---|---|
| open（张开） | `[1000, 1000, 1000, 1000, 1000, 500]` |
| close（握拳） | `[0, 0, 0, 0, 0, 0]` |
| peace（比 V） | `[0, 0, 1000, 1000, 0, 0]` |
| thumbs_up（点赞） | `[0, 0, 0, 0, 1000, 800]` |
| grab（抓握） | `[50, 50, 50, 100, 100, 0]` |

## 依赖项

- ROS Noetic（机器人控制、TF）
- PyBullet（物理仿真、逆运动学）
- YOLO-World / Ultralytics（开放词汇目标检测）
- AnyGrasp SDK（抓取位姿生成，需要许可证）
- pyrealsense2（Intel RealSense D455）
- CUDA/cuDNN（GPU 加速）
- scipy、numpy、open3d、PIL、termcolor

## 模型路径（在 `core/config.py` 中配置）

- YOLO-World：`dependence/yolo_world/yolov8x-worldv2.pt`
- AnyGrasp：`dependence/anygrasp_sdk/checkpoint_detection.tar`
- URDF（孪生模型）：`dependence/smart_pick_and_place_ws/src/rm_description/urdf/SingleArm/easy_single_arm_bullet.urdf`

## 流水线流程 (pick_and_place)

1. 从 kwargs 或 stdin 获取 JSON 命令
2. **抓取阶段** — 循环遍历 grasp1-4 位置：
   - 运动到抓取观测位姿
   - 采集 RGB-D 图像
   - 运行 AnyGrasp 生成抓取候选
   - 使用 YOLO-World 检测目标物体过滤抓取位姿（边界框重叠 + 20px 边距）
   - 将抓取位姿从相机坐标系变换到世界坐标系
   - 通过孪生推理生成无碰撞轨迹
   - 执行轨迹，闭合手，回到观测位姿
3. **放置阶段**（内联执行，不复用独立 skill 实例）：
   - 递送模式：经 2 个路径点平滑插值轨迹运动到 handover 位姿，张开手
   - 扔垃圾模式：运动到 `throw_to_trash_pose`，张开手
   - 放桌面模式：从 `desk_pose_1/2/3` 中随机选择，张开手
   - 普通模式：使用 YOLO-World 检测容器，从深度图计算 3D 位置，通过孪生推理生成轨迹

## 流水线流程 (fetch_from_user)

1. 从 kwargs 或 stdin 获取 JSON（仅需 `container` 字段）
2. 运动到 `handover_pose`，张开手
3. 等待用户放入物品（初始等待 1s + 重试等待 3s）
4. 闭合手，验证抓取（手指位置差值检测）
5. 执行放置（内联执行，模式同 pick_and_place）

## 开发原则

- **流水线执行中禁止跨 Skill 实例化**：高级 skill（如 `pick_and_place`、`fetch_from_user`）需要执行子任务（递送、扔垃圾、放桌面）时，必须使用 `self.control_arm` / `self.control_hand` 内联逻辑，而非创建新 Skill 实例。新实例会建立额外的 TCP 连接（arm 8010、hand 8000），引入延迟和连接冲突，破坏阶段间的无缝衔接。独立的 skill 类（`handover`、`trash`、`desk_place`）仅保留给独立 CLI 调用。
- **Skill 的 `run()` 必须先检查 `kwargs` 再回退 stdin**：`run_skill.py` 从 stdin 读取 JSON 后以 `kwargs` 传入。Skill 应先检查 `kwargs`（`if kwargs.get("field"): data = kwargs`），仅在 `kwargs` 为空时才回退到 `self.json_parser.get_command()`。否则 stdin 已被消费，parser 读不到数据。
- **位姿查询统一用 `Config.get_pose()`**：不要直接访问 `config.robot_config` 或 `config.default_traj_js`，使用 `config.get_pose(name)` 统一查询（先查顶层再查 `default_traj_js`，附带 `"J1" in pose` 类型校验）。
