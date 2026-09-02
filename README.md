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
│                          core/ (基础设施)                          │
│  config │ arm │ hand │ camera │ twin_client │ transforms          │
│  perception │ vlm │ json_input                                   │
└──────────┬──────────────┬────────────────────────────────────────┘
           │              │
     Socket Clients   Socket Servers
     (8010/8000/8020)  (ROS Nodes + Twin Server)
```

**三进程分布式架构：**
- **进程1 (start1.bash)**: ROS系统启动 — 机械臂驱动、相机节点、灵巧手节点
- **进程2 (start2.bash)**: 数字孪生推理服务器 — PyBullet物理仿真，IK求解与碰撞检测
- **主进程 (run_skill.py)**: 通过 CLI 调用技能

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

# 可选：启用 VLM 目标词扩展（使用 GLM-4.5V，不设置则自动使用原始目标词）
export GLM_API_TOKEN="你的智谱 API Token"

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
| `grasp` | 视觉抓取（VLM 目标词扩展 + YOLO-World + AnyGrasp + Twin 轨迹） |
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

## 位姿配置说明

`robot_config.json` 中的预定义位姿分两处存储：

| 存储位置 | 包含的位姿 | 查询方式 |
|----------|-----------|----------|
| `default_traj_js` 字段内 | `grasp1-4`、`place1-2` | `config.default_traj_js[name]` |
| JSON 顶层 | `handover_pose`、`get_ready_to_handover_*`、`throw_to_trash_pose`、`desk_pose_*` | `config.robot_config.get(name)` |

**统一查询方式**：始终使用 `config.get_pose(name)` 方法，该方法会先查顶层再查 `default_traj_js`，无需关心位姿存储在哪个位置。

### 统一机器人配置

根目录的 `robot_config.json` 是唯一机器人配置入口：

- `arms`：真实机械臂/夹爪的网络、相机、坐标系和预定义位姿。
- `shared`：双臂共享的服务地址、端口和标定参数。
- `robot_models`：仿真和 Twin 使用的 URDF 结构描述，包含 `left_gripper`、`right_gripper` 和 `dual_arm` 三套模型。

仿真代码通过 `Config.get_robot_model(name)` 读取模型，不再从 `rm_description/urdf/` 加载独立的机器人 JSON。RViz 配置、URDF 文件和录制轨迹仍是各自格式的资源文件，不属于机器人参数配置。

## 开发原则

- **流水线执行中禁止跨 Skill 实例化**：高级 skill（如 `pick_and_place`、`fetch_from_user`）需要执行子任务（递送、扔垃圾、放桌面）时，必须使用 `self.control_arm` / `self.control_hand` 内联逻辑，而非 `new Skill()` 创建新实例。新实例会建立额外的 TCP 连接（arm 8010、hand 8000），引入延迟和连接冲突，破坏阶段间的无缝衔接。
- **Skill 的 `run()` 方法必须先检查 `kwargs`**：`run_skill.py` 从 stdin 读取 JSON 后以 `kwargs` 传入。Skill 应先检查 `kwargs`（`if kwargs.get("field"): data = kwargs`），仅在 `kwargs` 为空时才回退到 `self.json_parser.get_command()`，否则 stdin 已被消费，parser 读不到数据。

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
│   ├── config.py             # 集中配置管理
│   ├── arm.py                # 机械臂 Socket 客户端 (:8010)
│   ├── hand.py               # 灵巧手 Socket 客户端 (:8000)
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

硬件资源（arm、hand、camera、twin、perception、vlm）通过 property 懒加载，首次访问时才建立连接。

## 添加新 Skill

1. 在 `skills/` 下创建 `my_skill.py`
2. 继承 `Skill`，添加 `@register_skill("my_skill")`
3. 实现 `run(self, **kwargs)`
4. 完成。调用：`echo '{"key":"value"}' | python run_skill.py my_skill`

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
