## Smart Pick-and-Place Framework (Based on ROS / Skill-DB Architecture)

```
conda activate anygrasp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib
```

---

### Framework structure:

```
Smart-Pick-and-Place-in-the-Real-World/
│
├── run_skill.py              # 统一 CLI 入口：argparse + JSON stdin → Skill Registry
│
├── skills/                   # Skill-DB（任务编排层，与硬件完全解耦）
│   ├── base.py               # Skill 基类：懒加载硬件属性（arm/hand/camera/twin/...）
│   ├── pick_and_place.py     # 高级 Skill：检测→抓取→放置 完整流水线
│   ├── fetch_from_user.py    # 高级 Skill：从用户手中接收物品→放置
│   ├── look_around.py        # 高级 Skill：多视角扫描 + VLM 场景分析
│   ├── capture_at_handover.py# 高级 Skill：handover 位拍照 + VLM 识别
│   ├── pose_execute.py       # 位姿/动作序列执行（支持手势预设）
│   ├── grasp.py              # 原子 Skill：视觉抓取（YOLO + AnyGrasp + Twin IK）
│   ├── place.py              # 原子 Skill：视觉放置（容器检测→放置轨迹）
│   ├── handover.py           # 原子 Skill：递交给用户
│   ├── trash.py              # 原子 Skill：扔垃圾
│   └── desk_place.py         # 原子 Skill：放桌面
│
├── core/                     # 共享基础设施 + 硬件抽象层
│   ├── config.py             # 集中配置管理：加载 robot_profile.json + robot_config.json
│   │                         #   提供关节无关接口：pose_to_list() / get_default_js_rad() / arm_struct_name
│   │
│   ├── drivers/              # 硬件抽象层（Driver ABC + 多硬件实现 + 工厂）
│   │   ├── arm_driver.py     #   ArmDriver ABC: connect / move_to_named_pose / execute_trajectory
│   │   ├── hand_driver.py    #   HandDriver ABC: connect / open / close / is_grasping / get_state
│   │   ├── rm75_driver.py    #   RM-75 7DOF 机械臂实现（TCP socket → :8010）
│   │   ├── inspire_driver.py #   Inspire 灵巧手实现（TCP socket → :8000）
│   │   ├── tianyi_arm_driver.py   # 天义右臂实现（ROS2 Topic → /arm/cmd_pos）
│   │   ├── tianyi_hand_driver.py  # 天义右手实现（ROS2 Topic → /inspire_hand/ctrl/right_hand）
│   │   └── factory.py        #   create_arm_driver() / create_hand_driver() ← robot_profile.json
│   │
│   ├── arm.py                # RM-75 TCP 客户端（底层，4字节大端长度前缀协议）
│   ├── hand.py               # Inspire TCP 客户端（底层，JSON 协议）
│   ├── camera.py             # 相机：RealSenseCapture (pyrealsense2) / TianyiCamera (WebSocket)
│   ├── twin_client.py        # 数字孪生客户端 → Twin IK 服务 (:8020)
│   ├── transforms.py         # ROS2 TF2 坐标变换 + 相机投影 + 工具函数
│   ├── perception.py         # YOLO-World 检测 + AnyGrasp 抓取（本地 SDK / WebSocket 远程）
│   ├── vlm.py                # GLM-4.5V 视觉语言模型 API 客户端
│   └── json_input.py         # JSON stdin 解析
│
├── dependence/               # 第三方依赖
│   ├── twin_inference/       #   数字孪生推理服务（PyBullet 仿真，IK/碰撞检测，端口 8020）
│   ├── anygrasp_sdk/         #   AnyGrasp 抓取检测 SDK
│   ├── yolo_world/           #   YOLO-World 模型 (yolov8x-worldv2.pt)
│   └── smart_pick_and_place_ws/  # ROS catkin 工作空间（机械臂/灵巧手/相机 bringup）
│
├── tools/                    # 开发工具
│   ├── pose_record.py        #   位姿录制（直连机械臂 SDK）
│   └── get_current_pose.py   #   读取当前关节角度
│
├── robot_profile.json        # 硬件描述配置（驱动类型、关节名/数、端口、URDF 路径、坐标系）
├── robot_config.json         # 任务位姿配置（grasp1-4、place1-2、handover、trash、desk 等，单位：度）
├── recorded_poses.json       # 录制的位姿库
│
├── robot_profile_tianyi.json      # 天义机器人 Profile（WebSocket 相机 + ROS2 Topic 驱动）
├── robot_config_tianyi.json       # 天义机器人任务位姿（关节名→度）
├── twin_robot_config_tianyi.json  # 天义机器人 Twin 结构体定义
│
├── start.bash                # 一键启动（ROS bringup + Twin IK 服务）
├── start1.bash               #   终端1：ROS 硬件 bringup（机械臂 :8010, 灵巧手 :8000）
└── start2.bash               #   终端2：Twin IK 服务 (:8020)
```

**硬件抽象层数据流：**

```
robot_profile.json          ← 硬件型号（driver: "rm75" / "tianyi"）+ 参数
    │
    ▼
core/config.py              ← 加载 profile，关节无关接口
    │
    ▼
core/drivers/factory.py     ← 根据 driver 字段选择实现类
    │
    ▼
skills/base.py              ← Skill 懒加载：self.arm → ArmDriver, self.hand → HandDriver
```

**跨臂适配 4 步完成（无需修改 skills/）：**
1. `robot_profile.json` — 修改关节名、DOF、driver 类型
2. `core/drivers/` — 新建实现类（实现 ArmDriver / HandDriver ABC）
3. `factory.py` — 注册新 driver
4. `robot_config.json` + URDF — 录制新臂位姿，提供 PyBullet 模型

---

### Development Example: Adding a new Skill

#### 1. Create skill file

Place the skill file in `skills/my_skill.py`:

```python
from skills.base import Skill, register_skill

@register_skill("my_skill")
class MySkill(Skill):
    def run(self, **kwargs):
        # 优先从 kwargs 读取参数（由 run_skill.py 传入）
        data = kwargs if kwargs else self.json_parser.get_command()
        
        # 通过懒加载属性访问硬件（自动根据 profile 选择驱动）
        self.arm.move_to_named_pose(self.config.get_pose("grasp1"))
        
        # 使用关节无关接口访问位姿
        js_rad = self.config.get_default_js_rad("grasp1")  # 度→弧度
        trajectory_resp = self.twin.generate_trajectory2(...)
        
        self.hand.close()
```

**注意**：Skill 层不要直接访问 `pose["J1"]` 或 `pose["shoulder_pitch_r_joint"]` 等硬编码关节名，始终使用 `config.get_pose()` / `config.pose_to_list()` / `config.get_default_js_rad()`。

#### 2. Done — no other files need to be changed

`skills/__init__.py` 会通过 import 自动触发 `@register_skill` 注册。

#### 3. Invoke the skill

```bash
echo '{"key":"value"}' | python run_skill.py my_skill
```

---

### Development Example: Adding a new Robot Arm

#### 1. Implement the ArmDriver ABC

Create `core/drivers/my_arm_driver.py`:

```python
from core.drivers.arm_driver import ArmDriver

class MyArmDriver(ArmDriver):
    def connect(self) -> bool:
        ...  # 建立与硬件的通信连接

    def close(self) -> None:
        ...  # 断开连接

    def move_to_named_pose(self, pose_dict: dict, speed: int = 30) -> bool:
        ...  # 将 pose_dict（度）转换为硬件指令并发送

    def execute_trajectory(self, trajectory, speed: int = 20) -> bool:
        ...  # 逐点发送轨迹（trajectory 为 [{joint_name: rad}, ...]）
```

#### 2. Register in factory.py

```python
# core/drivers/factory.py
from core.drivers.my_arm_driver import MyArmDriver

ARM_DRIVERS["my_arm"] = MyArmDriver
```

#### 3. Create profile and config

`robot_profile.json`:
```json
{
    "arm": {
        "driver": "my_arm",
        "num_joints": 7,
        "joint_names": ["joint1", "joint2", ...],
        "host": "192.168.x.x",
        "port": 8010
    },
    "hand": { ... },
    "twin": { ... },
    "frames": { ... }
}
```

#### 4. Provide URDF and record task poses

- 提供 PyBullet URDF 模型（仿真 IK 用）
- 在 `robot_config.json` 中录制任务位姿（单位：度）

---

### Hardware Setup

| 角色 | IP | 说明 |
|------|-----|------|
| 机械臂 (RM-75) | 192.168.1.19:8010 | TCP socket，4字节大端长度前缀协议 |
| 灵巧手 (Inspire) | 192.168.11.209:8000 | TCP socket，JSON 协议（Modbus 底层） |
| 相机 (RealSense D455) | USB local | pyrealsense2，640x480@30fps |
| Twin IK 服务 | 127.0.0.1:8020 | PyBullet 仿真，TCP socket |
| local | 192.168.3.15 | 主机（运行 skills + ROS + Twin） |
| server | 192.168.3.11 | AI 服务（AnyGrasp WS :8775, VLM API :8088） |
| orin | 192.168.3.16 | 边缘计算（相机 WS :8765） |

---

### Calling Skills

```bash
conda activate anygrasp
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World

# 查看所有可用 skill
python run_skill.py list

# 抓取物品放到容器里
echo '{"object":"orange","container":"green bowl"}' | python run_skill.py pick_and_place

# 把物品递给用户
echo '{"object":"bottle","container":"person"}'  | python run_skill.py pick_and_place

# 扔垃圾
echo '{"object":"wrapper","container":"trash"}'   | python run_skill.py pick_and_place

# 放到桌面
echo '{"object":"cup","container":"desk"}'        | python run_skill.py pick_and_place

# 从用户手中接收物品
echo '{"container":"trash"}' | python run_skill.py fetch_from_user

# 环顾桌面拍照
python run_skill.py look_around
```

**container 特殊值：**

| 值 | 模式 | 行为 |
|----|------|------|
| `"person"` | 递送 | 经中间路径点运动到 handover 位姿，松手 |
| `"trash"` / `"垃圾桶"` / `"bin"` | 扔垃圾 | 运动到 trash 位姿，松手 |
| `"desk"` / `"桌子"` / `"table"` | 放桌面 | 从 3 个预设桌面位姿随机选择 |
| 其他字符串 | 容器检测 | YOLO 检测容器位置 → 生成放置轨迹 |

---

### Startup Scripts (RM-75 Desktop Setup)

**start1.bash** — 终端1：ROS 硬件 bringup
```
构建 ROS catkin 工作空间，启动机械臂驱动节点、相机节点、灵巧手节点。
端口：灵巧手 :8000，机械臂 :8010
```

**start2.bash** — 终端2：Twin IK 服务
```
启动 PyBullet 数字孪生推理服务（端口 :8020）。
加载 URDF，提供 trajectory_generation / reachability_check / collision_check 服务。
```

**start.bash** — 一键启动（gnome-terminal）
```bash
./start.bash  # 自动打开 2 个终端分别执行 start1.bash 和 start2.bash
```

---

### Twin Service Types

| 类型 | 请求格式 | 响应 |
|------|---------|------|
| `reachability_check` | `{srv:"twin_inference", type, cnfg:{target_pose, current_js, struct}}` | `is_reached`, `delta_xyz`, `delta_rpy`, `is_collided` |
| `collision_check` | 同上 | 同上 |
| `trajectory_generation` | 同上（单目标） | `trajectory`（弧度）, `trajectory_ee`, `infos` |
| `trajectory_generation2` | 同上（多目标） | 同上 + `is_z_safe`, `unsafe_links` |
| `IK_calculation` | 同上 | 同 reachability_check |

**通信协议：**
- SEND：纯 JSON（无长度前缀）
- RECV：4 字节大端长度前缀 + JSON

---

### Dependencies

- **ROS Noetic** — 机器人控制和 TF（RM-75）；**ROS2 Humble** — 天义机器人
- **PyBullet** — 物理仿真、逆运动学、碰撞检测
- **YOLO-World (Ultralytics)** — 开放词汇目标检测
- **AnyGrasp SDK** — 抓取姿态生成（本地或 WebSocket 远程）
- **pyrealsense2** — RealSense D455 驱动；**OpenCV** — WebSocket 相机解码
- **CUDA / cuDNN** — GPU 加速
- scipy, numpy, open3d, PIL, termcolor
