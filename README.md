# Smart Pick and Place in the Real World

基于虚实结合双重推理架构桌面级智能机械臂平台

基于虚实结合双重推理架构的桌面级智能机械臂 Pick-and-Place 系统。系统通过 JSON 命令驱动，集成 YOLO-World 开放词汇目标检测、AnyGrasp 抓取姿态生成、PyBullet 仿真轨迹规划，实现真实环境下的智能抓取与放置。

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                     planner.py / fetch_from_user.py                 │
│              (主控：JSON命令 → 目标检测 → 抓取 → 放置)                │
└──────────┬──────────┬──────────┬──────────┬──────────┬──────────────┘
           │          │          │          │          │
     json_input.py  camera.py  utils.py  armcontroller.py  look_around.py
     (JSON解析)    (RealSense) (坐标变换)  (Socket机械臂控制)  (GLM-4.5V场景分析)
           │          │          │          │
           │          ▼          │          ▼
           │   anygrasp_sdk     │    Socket Servers
           │   (抓取姿态生成)    │    (8000灵巧手/8010机械臂)
           │          │          │          │
           │          ▼          ▼          ▼
           │   transformation.py    ROS Nodes (start1.bash)
           │   (ROS TF坐标变换)
           │          │
           └──────────┴──────────────────────┐
                      │                      │
                      ▼                      │
           twin_inference/twin.py            │
           (PyBullet仿真IK服务器, 端口8020)    │
```

**三进程分布式架构：**
- **进程1 (start1.bash)**: ROS系统启动 — 机械臂驱动、相机节点、灵巧手节点
- **进程2 (start2.bash)**: 数字孪生推理服务器 — PyBullet物理仿真，提供IK求解和碰撞检测
- **进程3 (start3.bash)**: 主控规划器 — 从stdin读取JSON命令执行任务

## 快速开始

### 前置条件

- ROS Noetic
- Conda 环境 `anygrasp` (Python 3.9)
- Intel RealSense D435 相机
- 7-DOF 机械臂 + Inspire 灵巧手
- CUDA / cuDNN

### 启动系统

```bash
# 1. 激活环境
conda activate anygrasp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib

# 2. 确保硬件连接
ping 192.168.1.19     # 机械臂IP
ping 192.168.11.210   # 灵巧手IP

# 3. 启动所有进程（在3个终端中分别启动，或使用下面的命令）
./start.bash
```

`start.bash` 会在3个 gnome-terminal 中分别启动 `start1.bash`、`start2.bash`、`start3.bash`。

### 发送任务命令

在 start3 窗口中输入 JSON 命令：

```bash
# 抓取橘子放到粉色盘子里
{"object": "orange", "container": "pink plate"}

# 抓取苹果或水果放到碗里
{"object": "apple,fruit", "container": "bowl"}

# 把瓶子递给用户
{"object": "bottle", "container": "person"}

# 把包装纸扔进垃圾桶
{"object": "wrapper", "container": "trash"}

# 放到桌子上
{"object": "cup", "container": "desk"}
```

## JSON 命令格式

### Pick and Place (`planner.py`)

```json
{
    "object": "orange",
    "container": "pink plate",
    "direction": "left"
}
```

| 字段 | 必填 | 说明 |
|------|------|------|
| `object` | 是 | 要抓取的物体名称，支持逗号分隔多类别（OR逻辑），直接传给 YOLO-World |
| `container` | 是 | 放置目标，可以是容器名称或特殊模式关键词 |
| `direction` | 否 | 方位提示 (left/right/middle/front/back)，暂未实现 |

**特殊放置模式：**

| container 值 | 模式 | 行为 |
|---|---|---|
| `"person"` | 递送 | 平滑轨迹经过中间点到达 handover 位姿，松手 |
| `"trash"` / `"垃圾桶"` / `"garbage"` / `"bin"` | 丢垃圾 | 移动到垃圾桶位姿，松手 |
| `"desk"` / `"桌子"` / `"table"` | 放桌面 | 随机选择3个预设桌面位姿之一 |

### Fetch from User (`fetch_from_user.py`)

从用户手中接收物品并放置，只需指定 `container`：

```json
{"container": "pink plate"}
```

流程：移动到 handover 位姿 → 张开手等待 → 用户放入物品 → 关闭手 → 放置到指定容器。

## 功能模块

### 核心模块

| 文件 | 功能 |
|------|------|
| `planner.py` | 主控管线：JSON → 目标检测 → 抓取 → 放置 |
| `fetch_from_user.py` | 反向管线：从用户接收物品 → 放置到容器 |
| `json_input.py` | 从 stdin 读取并解析 JSON 命令 |
| `armcontroller.py` | 机械臂控制，通过 Socket 发送关节空间指令 |
| `camera.py` | RealSense D435 RGB-D 图像采集 (640x480, 30fps) |
| `transformation.py` | ROS TF 坐标变换工具 (base_link ↔ cam_link) |
| `utils.py` | 相机投影、坐标变换、3D可视化 |
| `robot_config.json` | 预定义关节位姿、坐标系名称 |

### 辅助工具

| 文件 | 功能 |
|------|------|
| `look_around.py` | 遍历观测位置拍照，调用 GLM-4.5V 分析场景物品和空间关系 |
| `capture_at_handover.py` | 移动到 handover 观察位姿拍照，GLM-4.5V 识别用户手中物品 |
| `arm_pose_record_and_execute.py` | 位姿录制、回放和动作序列执行 |
| `get_current_pose.py` | 读取机械臂当前关节角度 |

### 数字孪生推理 (`twin_inference/`)

| 文件 | 功能 |
|------|------|
| `twin.py` | PyBullet 仿真服务器 (端口8020)，提供 IK 求解和轨迹生成 |
| `robot.py` | 机器人模型定义：`ErdaijiRobot`，包含 `Arm`/`Hand`/`Gripper`/`Head` 结构体 |
| `sim_world.py` | PyBullet 物理仿真环境 |
| `utils.py` | 变换矩阵计算、SLERP 四元数插值、可视化 |
| `p_utils.py` | PyBullet 关节/连杆/碰撞检测工具函数 |

### ROS 工作空间 (`smart_pick_and_place_ws/`)

| 路径 | 功能 |
|------|------|
| `src/rm_65_pkg/src/arm_75_bringup.py` | 机械臂 ROS 驱动节点 |
| `src/rm_65_pkg/src/mount_camera.py` | 相机标定/挂载节点 |
| `src/rm_65_pkg/src/inspire_hand_bringup.py` | Inspire 灵巧手 ROS 节点 |
| `src/rm_65_pkg/src/hand_controller_modbus.py` | 灵巧手 Modbus 控制节点 |
| `src/rm_description/urdf/SingleArm/` | URDF 模型和仿真配置 |

## 通信协议

系统使用 Socket 进行进程间通信：

| 端口 | 服务 | 协议 |
|------|------|------|
| 8000 | 灵巧手控制 | JSON: `{"src": "/left_hand/movement_control", "type": "set/get", "cmd": [...]}` |
| 8010 | 机械臂控制 | 4字节大端长度前缀 + JSON |
| 8020 | 数字孪生推理 | 请求: JSON; 响应: 4字节大端长度前缀 + JSON |

**机械臂控制指令格式 (端口 8010)：**
```json
{
    "srv": "/right_arm/movement_control",
    "cmd": [
        {"type": "start", "act": []},
        {"type": "js", "act": {"J1": 5.5, "J2": 38.4, ...}, "speed": 20, "block": true},
        {"type": "end", "act": []}
    ]
}
```

**注意：** `robot_config.json` 中的关节角度使用**角度制**，数字孪生返回的轨迹使用**弧度制**，`planner.py` 中会进行转换。

## 数字孪生服务类型

| 类型 | 说明 |
|------|------|
| `reachability_check` | 检查目标位姿是否可达（含碰撞检测） |
| `collision_check` | 碰撞检测 |
| `IK_calculation` | 逆运动学求解 |
| `trajectory_generation` | 单目标线性轨迹生成（含碰撞检测） |
| `trajectory_generation2` | 多目标线性轨迹生成（含碰撞检测和 Z 轴高度安全检查） |

## 预定义位姿

| 位姿名称 | 用途 |
|----------|------|
| `grasp1` ~ `grasp4` | 抓取观测位置（遍历搜索） |
| `place1`, `place2` | 放置后返回位置 |
| `handover_pose` | 递送物品给用户的位姿 |
| `get_ready_to_handover_1st`, `get_ready_to_handover_2nd` | 递送平滑轨迹中间路径点 |
| `throw_to_trash_pose` | 丢弃垃圾位姿 |
| `desk_pose_1/2/3` | 桌面放置位姿（随机选择） |
| `look_over_what_in_user_hand_pose` | 查看用户手中物品的相机位姿 |

## 位姿录制与回放

```bash
# 录制当前位姿
python3 arm_pose_record_and_execute.py record --name "home"

# 交互式录制
python3 arm_pose_record_and_execute.py record --interactive

# 执行预设位姿
python3 arm_pose_record_and_execute.py play --name home --speed 30

# 执行动作序列
python3 arm_pose_record_and_execute.py sequence --file sequence.json

# 列出已录制位姿
python3 arm_pose_record_and_execute.py list
```

**灵巧手手势预设：** `open`, `close`, `peace`, `rock`, `pointing`, `thumbs_up`, `ok`, `grab`

## 抓取流程

1. 从 stdin 读取 JSON 命令
2. **抓取阶段** — 遍历 grasp1-4 观测位置：
   - 移动到观测位姿，采集 RGB-D 图像
   - AnyGrasp 生成候选抓取姿态（top-50）
   - YOLO-World 检测目标物体，筛选与检测框重叠的抓取姿态（20px容差）
   - 坐标变换：相机坐标系 → 世界坐标系
   - 数字孪生生成无碰撞轨迹
   - 执行轨迹，关闭灵巧手，验证抓取（手指位置差检测）
3. **放置阶段**：
   - 递送模式：平滑插值轨迹经2个中间点到 handover 位姿
   - 垃圾桶/桌面模式：移动到预定义位姿，松手
   - 普通模式：YOLO-World 检测容器位置，深度图计算3D坐标，生成放置轨迹

## 目标检测

系统使用 YOLO-World 进行开放词汇目标检测，支持任意类别名称：
- 物体：orange, apple, lemon, pear, bottle, cup, banana, carrot 等
- 容器：bowl, plate, box, basket, tray 等
- 多类别语法：逗号分隔表示 OR 逻辑，如 `"object": "apple,orange,fruit"`

## 依赖

- **ROS Noetic** — 机器人控制和坐标变换
- **PyBullet** — 物理仿真和逆运动学
- **YOLO-World (Ultralytics)** — 开放词汇目标检测
- **AnyGrasp SDK** — 抓取姿态生成（需要许可证）
- **pyrealsense2** — Intel RealSense D435 驱动
- **CUDA / cuDNN** — GPU 加速
- **Robotic_Arm SDK** — RM65 机械臂直连控制（位姿录制器使用）
- scipy, numpy, open3d, PIL, matplotlib, termcolor
