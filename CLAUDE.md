# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指导。

## 项目概述

本项目为大创项目"基于虚实结合双重推理架构的桌面级智能机械臂平台"的机器人抓取放置系统。采用 **Skill-DB 架构**，通过统一 CLI 入口 `run_skill.py` 调用封装好的机器人技能，集成 YOLOE-26 开放词汇检测、AnyGrasp 抓取姿态生成、PyBullet 仿真轨迹规划，实现真实环境下的智能抓取与放置。

核心能力：
- **抓取放置 (Pick and Place)**：视觉驱动抓取物体并放置到容器中
- **递送 (Handover)**：将物体递送给用户或从用户手中接收物体
- **扔垃圾/放桌面**：使用预定义位姿将物品扔进垃圾桶或放到桌面上
- **环视 (Look Around)**：扫描工作空间，使用 GLM-4.5V VLM 分析场景
- **位姿录制 (Pose Recording)**：录制和回放机械臂位姿及动作序列

## 运行系统

系统需要通过 `start.bash` 启动 2 个并发进程：

```bash
./start.bash  # 在 2 个 gnome-terminal 中启动 ROS bringup 和 Twin IK 服务
```

- **终端1**：ROS bringup — 构建工作空间并启动 ROS 节点（双臂驱动、双相机）
- **终端2**：孪生推理服务 — PyBullet 仿真用于逆运动学/轨迹生成（端口 8020）

**Conda 环境**：`anygrasp`（Python 3.9）

**cuDNN 库路径**（AnyGrasp 运行必需）：
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib
```

**硬件 IP 地址**：
- 左臂（夹爪 Robotiq 85）：192.168.1.19，Socket 端口 8010（机械臂）、8002（夹爪）
- 右臂（夹爪 Robotiq 85）：192.168.1.18，Socket 端口 8011（机械臂）、8001（夹爪）

> 历史：左臂末端执行器曾为 Inspire 灵巧手（端口 8000，ROS bringup），2026-07-12 更换为右臂同款 Robotiq 85 夹爪。灵巧手代码（`core/hand.py`、`core/arm_side.py`、dexterous 分支）已彻底删除；ROS 侧 `inspire_hand_bringup.py` / `hand_controller_modbus.py` 文件仍在但已从 bringup.launch 移除。

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                       run_skill.py (统一入口)                      │
│            argparse + JSON stdin → Skill Registry → skill.run()  │
└──────────┬───────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────────────────────────────────────────────────────┐
│                  skills/ (两层：单步 + 组合)                        │
│  组合 skill：pick_and_place │ fetch_from_user │ grasp_to_drawer   │
│              look_around │ capture_at_handover                   │
│  单步 skill：grasp │ place │ dual_handover │ handover             │
│              desk_place │ pose_execute                           │
└──────────┬───────────────────────────────────────────────────────┘
           │  委托（同实例，不建新连接）
           ▼
┌──────────────────────────────────────────────────────────────────┐
│            core/ 流水线（GraspPipeline / PlacePipeline /           │
│            dual_handover 回放引擎）+ 基础设施                       │
│  基础设施：config │ arm │ sim_arm │ gripper │ sim_gripper         │
│  camera │ sim_camera │ twin_client │ transforms │ perception      │
│  │ vlm │ json_input │ transition │ anygrasp_client                │
└──────────┬──────────────────────┬────────────────────────────────┘
           │                      │
     Socket Clients         Socket Servers
     (8010/8011/8001/8002/  (ROS Nodes + Twin(8020/8021)
      8020/8021/8030)        + Gripper Servers + AnyGrasp(8030)
                             + SimServer)
```

**运行模式**：`start.bash` 启动真实硬件（ROS bringup + Twin IK）；`start_sim.bash` 启动 PyBullet 仿真（SimServer，`Config.sim_mode` 下 arm/gripper/camera 自动分发到 `core/sim_*.py` 客户端）。

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

# 右臂直接递给用户（不经过左臂）
echo '{"speed":15}' | python run_skill.py right_give_to_user

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

- `object`（必需）：要抓取的物体名称（建议使用**英文**，YOLOE-26 文本提示对英文支持最好），支持逗号分隔的多个类别（OR 逻辑）
- `container`（必需）：放置目标容器，或特殊模式关键字
- `direction`（可选）：空间提示（尚未实现）

### 特殊容器模式
| 容器值 | 模式 | 行为 |
|---|---|---|
| `"person"` | 递送 | 默认左臂递交；指定 `side:"right"` 时执行右臂 `handover_pose`，不经过左臂 |
| `"trash"`、`"垃圾桶"`、`"garbage"`、`"bin"` | 扔垃圾 | 运动到扔垃圾位姿，松手 |
| `"desk"`、`"桌子"`、`"table"` | 放桌面 | 从 3 个预定义桌面位姿中随机选择，松手 |

## 项目结构

```
Smart-Pick-and-Place-in-the-Real-World/
├── run_skill.py              # 统一 CLI 入口
├── robot_config.json         # 机器人配置（arms.<side> 嵌套：关节位姿、标定、抓取参数）
│
├── skills/                   # 两层 Skill-DB
│   ├── base.py               # Skill 基类：注册机制、懒加载硬件、arm_for/gripper_for 分发
│   ├── __init__.py           # 导入所有 skill 触发注册
│   ├── pick_and_place.py     # 组合：抓取+放置（放置委托 place_pipeline，双臂交接委托 dual_handover）
│   ├── fetch_from_user.py    # 组合：从用户接收→放置（同上委托）
│   ├── grasp_to_drawer.py    # 组合：左臂抓取 ∥ 右臂开抽屉→双臂交接→放抽屉→关抽屉
│   ├── look_around.py        # 组合：场景扫描 + VLM 分析
│   ├── capture_at_handover.py# 组合：handover 拍照
│   ├── grasp.py              # 单步：视觉抓取（薄壳，实现在 core/grasp_pipeline.py）
│   ├── place.py              # 单步：视觉放置（薄壳，实现在 core/place_pipeline.py）
│   ├── dual_handover.py      # 单步：双臂交接位姿序列回放（左→右 / 右→左）
│   ├── handover.py           # 单步：递交给用户
│   ├── right_give_to_user.py # 单步：右臂命名位姿递给用户
│   ├── desk_place.py         # 单步：放桌面
│   └── pose_execute.py       # 位姿/动作序列/轨迹回放执行（支持手势、并行）
│
├── core/                     # 流水线 + 共享基础设施
│   ├── grasp_pipeline.py     # 抓取流水线（AnyGrasp 候选→评分→Twin 规划→执行→恢复）
│   ├── place_pipeline.py     # 放置流水线（VLM/YOLO 容器检测→安全区域→两段式→验证）
│   ├── dual_handover.py      # 双臂定时交接轨迹回放引擎（tools/ 同名脚本为 CLI shim）
│   ├── config.py             # 集中配置管理（get_pose(name, side) 统一查询）
│   ├── arm.py / sim_arm.py   # 机械臂客户端（真实 :8010/8011 / PyBullet 仿真）
│   ├── gripper.py / sim_gripper.py     # Robotiq 85 夹爪客户端（真实 8001/8002 / 仿真 mimic）
│   ├── camera.py / sim_camera.py       # RGB-D 采集（真实 RealSense / 仿真渲染）
│   ├── sim_utils.py          # 仿真辅助（sim_mode 分发、坐标换算）
│   ├── transition.py         # 真/仿真切换辅助
│   ├── anygrasp_client.py    # AnyGrasp 长驻服务客户端 (:8030)
│   ├── twin_client.py        # 数字孪生客户端 (:8020 左 / :8021 右)
│   ├── transforms.py         # ROS TF 坐标变换 + 工具函数
│   ├── perception.py         # YOLOE-26 + AnyGrasp 封装
│   ├── vlm.py                # GLM-4.5V 视觉语言模型客户端
│   └── json_input.py         # JSON stdin 解析
│
├── recorded_poses/           # 双臂示教位姿库（pose_execute 使用）
│   ├── left.json             # 左臂 18 个位姿（grasp1-4、place1-2、handover 等）
│   └── right.json            # 右臂 19 个位姿（抽屉序列、desk_front 等）
├── recorded_trajectories/    # SDK 轨迹录制（left/ right/ dual/）
├── recorded_sequences/       # 动作序列（如 drawer_cycle.json）
├── configs/                  # 实验配置（dual_vlm_sorting 等）
├── tests/                    # pytest 测试（python3 -m pytest tests/）
│
├── tools/                    # 开发工具（非 skill）
│   ├── pose_record.py        # 位姿录制（直连机械臂 SDK）
│   ├── record_sequence.py    # 动作序列录制
│   ├── play_dual_handover.py # 双臂交接回放 CLI（调 core/dual_handover）
│   ├── record_dual_handover.py # 双臂交接轨迹录制
│   ├── calibrate_arms.py     # 双臂/相机标定
│   └── get_current_pose.py   # 读取当前关节角度
│
├── dependence/               # 第三方依赖
│   ├── twin_inference/       # 数字孪生推理（twin.py :8020/8021；sim_server.py 双臂 SimServer）
│   ├── anygrasp_server/      # AnyGrasp 长驻服务 (:8030)
│   ├── anygrasp_sdk/         # AnyGrasp 抓取检测 SDK
│   ├── yolo_world/           # YOLOE-26 模型
│   └── smart_pick_and_place_ws/ # ROS catkin 工作空间
│
├── start.bash                # 一键启动真实硬件（ROS + Twin IK）
└── start_sim.bash            # 一键启动 PyBullet 仿真（SimServer）
```

## 关键文件

### Skills (`skills/`)
| 文件 | 用途 |
|---|---|
| `base.py` | Skill 基类：注册机制、懒加载硬件（`arm_for`/`gripper_for`/`twin_for`/`get_camera` 按 side 分发，sim_mode 自动切仿真客户端）、`grasp_pipeline`/`place_pipeline` 委托属性 |
| `pick_and_place.py` | 组合流水线：视觉抓取 → 放置路由（视觉容器/desk→左臂；drawer/cabinet→右臂固定位姿；trash/person 跟随 side） |
| `fetch_from_user.py` | 组合流水线：handover 位接收 → 放置 |
| `grasp_to_drawer.py` | 组合流水线：左臂抓取 ∥ 右臂开抽屉 → 双臂交接 → 右臂放抽屉 → 关抽屉 |
| `dual_handover.py` | 单步：双臂定时交接轨迹回放，`direction` 决定夹爪事件角色 |
| `grasp.py` | 单步：视觉抓取（委托 `core/grasp_pipeline.py`） |
| `place.py` | 单步：视觉放置（委托 `core/place_pipeline.py`） |
| `pose_execute.py` | 位姿/序列/轨迹回放，支持手势（open/close）与 parallel 并行 |

### Core (`core/`)
| 文件 | 用途 |
|---|---|
| `grasp_pipeline.py` | `GraspPipeline(skill)`：候选生成/评分/Twin 规划/执行/失败恢复，双臂统一入口 `visual_grasp()` |
| `place_pipeline.py` | `PlacePipeline(skill)`：VLM+YOLO 容器检测、内部安全区域采样、两段式放置（预放置位→复观测校正→下降）、释放与放置后验证 |
| `dual_handover.py` | 双臂定时交接回放引擎：多进程 SDK 直驱 + 定时夹爪事件 |
| `handover_pipeline.py` | 用户交接路由：左右臂 named-pose、双臂交接 |
| `config.py` | 配置加载，`get_pose(name, side)` / `get_arm_config(side)` 统一查询 |
| `arm.py` / `sim_arm.py` | 机械臂 TCP 客户端（4 字节大端长度前缀协议） / 仿真客户端 |
| `gripper.py` / `sim_gripper.py` | Robotiq 85 夹爪客户端（8001 右 / 8002 左；soft close 检测 gOBJ） / 仿真 mimic 客户端 |
| `camera.py` / `sim_camera.py` | RealSense RGB-D（640x480, 30fps） / 仿真渲染 |
| `anygrasp_client.py` | AnyGrasp 长驻服务客户端（:8030，server 见 `dependence/anygrasp_server/`） |
| `twin_client.py` | 孪生推理 TCP 客户端（左 :8020 / 右 :8021） |
| `transforms.py` | ROS TF 坐标变换、相机投影、3D 可视化工具函数 |
| `perception.py` | YOLOE-26 + AnyGrasp 封装 |
| `vlm.py` | GLM-4.5V 视觉语言模型 API 客户端 |
| `transition.py` / `sim_utils.py` | 真/仿真切换与仿真辅助 |

### 孪生推理系统 (`dependence/twin_inference/`)
| 文件 | 用途 |
|---|---|
| `twin.py` | PyBullet socket 服务（端口 8020），逆运动学求解和轨迹生成 |
| `sim_server.py` | 双臂仿真服务（SimServer）：加载 URDF + 场景，socket 执行轨迹/位姿、mimic 夹爪、RGB-D 渲染（`start_sim.bash` 启动） |
| `robot.py` | 机器人模型：`ErdaijiRobot` 类，`Arm`/`Hand`/`Gripper`/`Head` 结构体 |
| `sim_world.py` | PyBullet 物理仿真环境 |
| `utils.py` | 变换矩阵、SLERP 插值、可视化辅助 |
| `p_utils.py` | PyBullet 关节/连杆/碰撞工具函数 |

### ROS 工作空间 (`dependence/smart_pick_and_place_ws/`)
| 文件 | 用途 |
|---|---|
| `src/rm_65_pkg/src/arm_75_bringup.py` | 机械臂 ROS bringup 节点 |
| `src/rm_65_pkg/src/mount_camera.py` | 相机安装/标定节点 |
| `src/rm_65_pkg/src/inspire_hand_bringup.py` | Inspire 灵巧手 ROS 节点（**已停用**，左臂换夹爪后从 bringup.launch 移除；保留文件作为回归路径） |
| `src/rm_65_pkg/src/hand_controller_modbus.py` | 通过 Modbus 协议控制灵巧手（同样停用） |
| `src/rm_description/urdf/LeftArm/` | 仿真用 URDF 模型和机器人配置 |

## Socket 通信协议

| 端口 | 服务 | 协议 | 消息格式 |
|---|---|---|---|
| 8001 | 右臂夹爪（Robotiq 85） | TCP | JSON：`{"src": "/right_gripper/movement_control", "type": "set"/"get", "cmd": [v, v]}`（v∈0..1000，1000=张开，0=闭合） |
| 8002 | 左臂夹爪（Robotiq 85） | TCP | 同上，`src` 为 `/left_gripper/movement_control` |
| 8010 | 左臂机械臂控制 | TCP | 4 字节大端长度前缀 + JSON：`{"srv": "/right_arm/movement_control", "cmd": [{"type": "start"}, {"type": "js", "act": {...}, "speed": N, "block": bool}, {"type": "end"}]}` |
| 8011 | 右臂机械臂控制 | TCP | 同上 |
| 8020 | 左臂孪生推理 | TCP | 请求：纯 JSON；响应：4 字节大端长度前缀 + JSON |
| 8021 | 右臂孪生推理 | TCP | 同上 |
| 8030 | AnyGrasp 抓取检测 | TCP | 二进制：请求 4 字节 BE 长度前缀 + (JSON header + `\n` + depth raw + rgb raw)；响应 4 字节 BE 长度前缀 + JSON `{"poses": [...], "error": null}`。Server 长驻，模型仅 load_net 一次。 |

**重要**：`robot_config.json` 和机械臂命令中的关节角度使用**角度制（度）**。孪生推理返回的轨迹使用**弧度制**，skill 内部会进行转换（除以 π × 180）。

## 位姿配置说明

位姿有两套存储，用途不同：

**1. `robot_config.json`**（流水线技能使用，按 `arms.<side>` 嵌套，用 `Config.get_pose(name, side)` 查询）：

| 臂 | `default_traj_js` 内 | 臂级位姿 |
|---|---|---|
| left | `grasp1-4` | `place1/2`、`handover_pose`、`get_ready_to_handover_1st/2nd`、`throw_to_trash_pose`、`desk_pose_1/2/3`、`home`、`observe_right_arm` 等 |
| right | `desk_front`、`desk_side`、`ready_open_drawer1` | `handover_pose`、`throw_to_trash_pose`、`drawer_1_placement`、`home`、`observe_left_arm` 等 |

**2. `recorded_poses/{left,right}.json`**（`pose_execute` skill 使用）：示教录制的位姿库，左臂 18 个、右臂 19 个，含抽屉序列中间态（`home_start`/`approach_drawer`/`grasp_handle`/`drawer_open` 等）和双臂交接 preset。`_resolve_action_arm` 按库归属自动路由臂，错臂请求会被拒绝。

**轨迹录制**在 `recorded_trajectories/{left,right,dual}/`：抽屉开/关（SDK 轨迹回放，硬编码右臂）、双臂定时交接（`dual_handover_timed_*`，左右方向共用同一航点，仅夹爪事件角色互换）等。

## 孪生推理服务类型

| 类型 | 描述 | 响应 |
|---|---|---|
| `reachability_check` | 检查目标位姿是否可达 + 碰撞检测 | `is_reached`、`delta_xyz`、`delta_rpy`、`is_collided` |
| `collision_check` | 与可达性检查相同 | 同上 |
| `IK_calculation` | 与可达性检查相同 | 同上 |
| `trajectory_generation` | 单目标线性轨迹 + 碰撞检测 | `trajectory`（弧度）、`trajectory_ee`、`infos` |
| `trajectory_generation2` | 多目标线性轨迹 + Z 轴高度安全检查 | 同上 + `is_z_safe`、`unsafe_links` |

## 检测与抓取

- **物体检测**：YOLOE-26（yoloe-26s-seg.pt）— 开放词汇文本提示（建议将中文口语名先翻成英文，如 桃子→`peach`、瓶子→`bottle`、杯子→`cup`）
- **抓取检测**：AnyGrasp SDK — 生成 top-50 抓取候选，使用 YOLO 检测边界框（20px 边距）过滤
- **多类别**：逗号分隔值使用 OR 逻辑（如 `"apple,orange,fruit"`）

## 末端执行器控制

两条臂末端均为 Robotiq 85 平行夹爪，统一走 `core/gripper.py` 的 `GripperClient`（接口：`open/close/get_state/is_grasping/is_fully_open/get_finger_deviation`）。`skills/base.py` 的 `hand` property / `gripper_for(side)` 按 `arms.<side>.hand_type` 字段分发（当前均为 `"gripper"`）。

**抓取检测**：`is_grasping()` 使用 soft close（force=20）+ 服务端 `object_detected`（gOBJ）标志，比位置阈值可靠（空闭合会到达机械限位 pos≈230）。

**夹爪指令格式**：2 值数组 `[v, v]`（0..1000，1000=完全张开，0=完全闭合）。

`pose_execute` skill 的手势预设仅支持 `"open"` / `"close"` 两种语义；其他预设（peace、thumbs_up 等）会被映射到 close 并打印警告。

## 依赖项

- ROS Noetic（机器人控制、TF）
- PyBullet（物理仿真、逆运动学）
- YOLOE-26 / Ultralytics（开放词汇目标检测与分割）
- AnyGrasp SDK（抓取位姿生成，需要许可证）
- pyrealsense2（Intel RealSense D455）
- CUDA/cuDNN（GPU 加速）
- scipy、numpy、open3d、PIL、termcolor

## 模型路径（在 `core/config.py` 中配置）

- YOLOE-26：`dependence/yolo_world/yoloe-26s-seg.pt`
- AnyGrasp：`dependence/anygrasp_sdk/checkpoint_detection.tar`
- URDF（孪生模型）：`dependence/smart_pick_and_place_ws/src/rm_description/urdf/LeftArm/left_arm_bullet.urdf`

## 流水线流程 (pick_and_place)

1. 从 kwargs 或 stdin 获取 JSON 命令（`object`、`container`、`side` 默认 left、`location` 默认 desk_front）
2. **抓取阶段**：统一走 `visual_grasp()`（`core/grasp_pipeline.py`）— 循环观测位姿 → AnyGrasp 候选 → 评分（anygrasp/twin/宽度/高度/角度加权）→ Twin 规划（高位预抓取 → 低位预抓取 → 执行）→ 夹爪闭合
3. **放置阶段**（按容器类型路由，见上表）：
   - `"person"`：默认左臂递送；`side:"right"` 时执行右臂 `handover_pose`，不经过左臂
   - `"trash"`：左臂先双臂交接给右臂再扔 / 右臂直接扔
   - `"desk"`：右臂抓取时先交接给左臂，随机 `desk_pose_1/2/3`
   - drawer/cabinet：**强制右臂**放置（窄容器），左臂抓取时先双臂交接；只放入 `drawer_1_placement`，不会自动开/关抽屉（开抽屉用 `grasp_to_drawer` 或 `pose_execute`）
   - 视觉容器（碗/盘等）：**左臂**相机执行；右臂抓取时先交接给左臂。走 `place_pipeline`：VLM/YOLO 检测容器 → 内部安全区域采样多候选 → 预放置位 → 复观测校正 XYZ → 下降释放 → 放置后视觉验证

## 流水线流程 (fetch_from_user)

1. 从 kwargs 或 stdin 获取 JSON（仅需 `container` 字段）
2. 运动到 `handover_pose`，张开手
3. 等待用户放入物品（初始等待 1s + 重试等待 3s）
4. 闭合手，`check_grasping_object()` 验证抓取
5. 执行放置（trash/desk 内联，视觉容器委托 `place_pipeline`）

## 流水线流程 (grasp_to_drawer)

1. 左臂 `visual_grasp` ∥ 右臂以默认 `0.5x` 回放 `open_drawer` SDK 轨迹（并行线程）
2. 双臂定时交接（`core/dual_handover` 回放，左→右）
3. 右臂 → `drawer_1_placement` 松开夹爪
4. 右臂退回 home（清出抽屉内部）
5. 右臂以默认 `0.5x` 回放 `close_drawer` SDK 轨迹

## 位姿录制

使用 `tools/pose_record.py` 直连机械臂 SDK 读取当前关节角度，按臂保存到 `recorded_poses/{left,right}.json`（`pose_execute` 的位姿源）。

```bash
conda activate anygrasp

# 录制左臂位姿（默认）
python3 tools/pose_record.py record --name grasp1 --desc "抓取位1"

# 交互式连续录制左臂
python3 tools/pose_record.py record -i

# 录制右臂位姿（--arm right 指定 192.168.1.18）
python3 tools/pose_record.py record --name grasp1 --arm right
python3 tools/pose_record.py record -i --arm right

# 查看 / 删除已录制位姿
python3 tools/pose_record.py list
python3 tools/pose_record.py delete --name grasp1
```

**操作流程**：开启示教模式 → 手动将机械臂拖到目标位姿 → 运行录制命令 → 位姿自动保存。录制完成后需手动将位姿复制到 `robot_config.json` 对应的 left/right arm 配置中。

## 开发原则

- **两层 Skill 架构，逻辑住在 core 流水线**：单步 skill（grasp、place、dual_handover、handover、desk_place）是薄壳；组合 skill（pick_and_place、fetch_from_user、grasp_to_drawer）通过**组合**而非继承复用它们。抓取/放置的实现分别在 `core/grasp_pipeline.py` 和 `core/place_pipeline.py`，以 `GraspPipeline(skill)` / `PlacePipeline(skill)` 委托类持有，经 `__getattr__` 回落到同一 skill 实例取硬件。**不要把流水线逻辑加回 `skills/base.py`**，也不要在组合 skill 里重写放置/抓取逻辑。
- **流水线执行中禁止跨 Skill 实例化**：组合 skill 需要子能力时，调用对应 core 模块或 `self.xxx_pipeline`（同实例复用连接），而非创建新 Skill 实例——新实例可能建立额外 TCP 连接（左臂 8010、左夹爪 8002 等），引入延迟和连接冲突。例外：`grasp_to_drawer` 的抽屉轨迹回放经 `PoseExecuteSkill` 轻量实例（`_play_trajectory` 自建独立 SDK socket，不碰 skill 连接）。
- **Skill 的 `run()` 必须先检查 `kwargs` 再回退 stdin**：`run_skill.py` 从 stdin 读取 JSON 后以 `kwargs` 传入。Skill 应先检查 `kwargs`（`if kwargs.get("field"): data = kwargs`），仅在 `kwargs` 为空时才回退到 `self.json_parser.get_command()`。否则 stdin 已被消费，parser 读不到数据。
- **位姿查询统一用 `Config.get_pose(name, side)`**：不要直接访问 `config.robot_config` 或 `config.default_traj_js`，使用 `get_pose()` 统一查询（按臂嵌套查找，附带 `"J1" in pose` 类型校验）。
- **双臂交接只有一个实现**：`core/dual_handover.py` 的 `play(name, speed, require_confirmation, direction, skill)`——4 步纯位姿复现序列（预置位并行→右臂接近→接收臂闭合/交出臂打开→回预置位→双臂回 home），位姿来自 `recorded_poses/` 的 `right_to_left_handover_*` 与 `home`。左右方向共用同一位姿组，`direction`（`left_to_right`/`right_to_left`，必须显式）只决定夹爪事件角色（接收臂 close、交出臂 open）。调用方传入 `skill=self` 以复用其 arm_for/gripper_for 连接（sim 自动路由）。
