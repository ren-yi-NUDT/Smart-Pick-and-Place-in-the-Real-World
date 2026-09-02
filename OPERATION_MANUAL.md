# 操作手册

桌面级智能机械臂平台的运行操作指南，覆盖**真机模式**与**虚拟仿真模式**两种运行方式。

> 架构、协议、硬件细节见 [README.md](README.md)；skill 命令速查见 [COMMANDS.md](COMMANDS.md)。

---

## 1. 系统概述

本项目通过统一入口 `run_skill.py` 调用封装好的机器人技能（Skill-DB 架构），实现视觉抓取、放置、递送、扔垃圾、开关抽屉等操作。

系统有两种运行模式：

| 模式 | 后端 | 触发方式 | 适用场景 |
|------|------|----------|----------|
| **真机模式** | 真实机械臂 + ROS + Twin IK | 默认 | 有硬件、实际作业 |
| **虚拟仿真模式** | PyBullet 物理仿真（SimServer :8031） | 环境变量 `SIM_MODE=1` | 无硬件、开发调试、演示 |

仿真模式下，机械臂 / 夹爪 / 相机信号被路由到 PyBullet 数字孪生，无需连接真机即可跑通位姿类技能。

---

## 2. 环境准备

### 2.1 依赖

- **Conda 环境**：`anygrasp`（Python 3.9）
- **ROS Noetic**：`/opt/ros/noetic/setup.bash`
- **PyBullet**：仿真后端（仿真模式必需）
- **xfce4-terminal**：启动脚本用它开多标签终端窗口
- **CUDA / cuDNN**：AnyGrasp 抓取检测必需（真机视觉流程）

### 2.2 激活环境

```bash
conda activate anygrasp

# AnyGrasp 运行时必需的 cuDNN 路径（仅真机视觉流程需要）
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib
```

### 2.3 检查启动脚本依赖

```bash
# 仿真/真机启动脚本都依赖 xfce4-terminal
command -v xfce4-terminal || echo "需要安装：sudo apt install xfce4-terminal"
```

---

## 3. 真机模式

### 3.1 硬件与端口

| 设备 | 型号 | 连接 | 服务端口 |
|------|------|------|----------|
| 左臂 | RM 系列 7-DOF | 192.168.1.19 | 机械臂 8010 |
| 右臂 | RM 系列 7-DOF | 192.168.1.18 | 机械臂 8011 |
| 右臂夹爪 | Robotiq 85 | /dev/ttyUSB0, slave=9 | 8001 |
| 左臂夹爪 | Robotiq 85 | /dev/ttyUSB1, slave=1 | 8002 |
| 孪生推理（左） | Twin IK | 本地 | 8020 |
| 孪生推理（右） | Twin IK | 本地 | 8021 |
| AnyGrasp | 抓取检测 | 本地 | 8030 |

### 3.2 启动服务

```bash
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World
bash start.bash
```

脚本会弹出 1 个 xfce4-terminal 窗口，含 6 个标签页：

| 标签 | 服务 | 说明 |
|------|------|------|
| ROS Bringup | `bringup.launch` | 双臂驱动、双相机（左臂 8010、右臂 8011） |
| Twin IK (left) | `twin.py --side left` | 左臂孪生推理 :8020 |
| Twin IK (right) | `twin.py --side right` | 右臂孪生推理 :8021 |
| Gripper R | `server.py --port 8001` | 右臂夹爪（/dev/ttyUSB0, slave 9） |
| Gripper L | `server.py --port 8002` | 左臂夹爪（/dev/ttyUSB1, slave 1） |
| AnyGrasp Server | `anygrasp_server.py` | 抓取检测 :8030 |

### 3.3 运行技能

服务就绪后，在仓库根目录直接调用：

```bash
# 查看所有 skill
python run_skill.py list

# 抓取 + 放置
echo '{"object":"orange","container":"green bowl"}' | python run_skill.py pick_and_place

# 递送 / 扔垃圾 / 放桌面
echo '{"object":"bottle","container":"person"}' | python run_skill.py pick_and_place
echo '{"object":"wrapper","container":"trash"}' | python run_skill.py pick_and_place
echo '{"object":"cup","container":"desk"}' | python run_skill.py pick_and_place

# 从用户手中接收物品
echo '{"container":"pink plate"}' | python run_skill.py fetch_from_user
```

完整命令速查见 [COMMANDS.md](COMMANDS.md)。

---

## 4. 虚拟仿真模式

### 4.1 原理

设置环境变量 `SIM_MODE=1` 后，`skills/base.py` 会自动把以下客户端替换为仿真客户端（连 PyBullet SimServer `127.0.0.1:8031`）：

| 资源 | 真机客户端 | 仿真客户端 |
|------|-----------|-----------|
| 机械臂 | `core/arm.py` (8010/8011) | `core/sim_arm.py` → :8031 |
| 夹爪 | `core/gripper.py` (8001/8002) | `core/sim_gripper.py` → :8031 |
| 相机 | `core/camera.py` (RealSense) | `core/sim_camera.py` → :8031 |

SimServer 加载**双臂** URDF（左臂 + 右臂），所有命令带 `side` 字段路由到对应机械臂。

> 注意：**感知（YOLO-World / AnyGrasp）与孪生 IK 暂未路由到仿真**，因此依赖视觉检测/逆解轨迹的技能（pick_and_place、grasp、place、fetch_from_user、grasp_to_drawer）目前仍需真机服务，属 Phase 2 范围。

### 4.2 启动仿真

```bash
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World
bash start_sim.bash
```

脚本弹出 xfce4-terminal 窗口，含 2 个标签页：

| 标签 | 服务 | 说明 |
|------|------|------|
| roscore | ROS master | SimServer 依赖 `rospy.init_node`，必须先起 |
| PyBullet SimServer | `sim_server.py --port 8031` | 双臂仿真（GUI） |

**无图形界面环境**可改用 headless 模式：

```bash
cd dependence/twin_inference
source /opt/ros/noetic/setup.bash
conda activate anygrasp
python3 sim_server.py --novis --port 8031
```

### 4.3 运行技能（仿真）

在命令前加 `SIM_MODE=1`：

```bash
# 扔垃圾
SIM_MODE=1 python run_skill.py trash

# 递交给用户
SIM_MODE=1 python run_skill.py handover

# 放桌面
SIM_MODE=1 python run_skill.py desk_place

# 位姿回放（左臂 home）
echo '{"command":"play","name":"home","arm":"left"}' | SIM_MODE=1 python run_skill.py pose_execute

# 开抽屉 / 关抽屉（右臂轨迹回放）
echo '{"command":"open_drawer"}' | SIM_MODE=1 python run_skill.py pose_execute
echo '{"command":"close_drawer"}' | SIM_MODE=1 python run_skill.py pose_execute
```

### 4.4 仿真当前支持范围

**✅ 已跑通（位姿/轨迹类）**

| Skill | 说明 |
|-------|------|
| `trash` | 移动到垃圾桶位姿松手 |
| `handover` | 移动到 handover 位姿 |
| `desk_place` | 随机选桌面位姿 |
| `pose_execute` | 位姿回放 + 开关抽屉（右臂轨迹） |

**⏳ 待 Phase 2（视觉/逆解类，依赖真机感知或孪生 IK）**

| Skill | 依赖 |
|-------|------|
| `pick_and_place` | YOLO-World + AnyGrasp + Twin IK |
| `grasp` | YOLO-World + AnyGrasp + Twin IK |
| `place` | YOLO-World + Twin IK |
| `fetch_from_user` | Twin IK |
| `grasp_to_drawer` | 视觉抓取 + 右臂 SDK |
| `look_around` / `capture_at_handover` | VLM（云 API，待验证） |

---

## 5. Skill 速查

### 5.1 通用

```bash
python run_skill.py list    # 列出所有已注册 skill
```

### 5.2 抓取与放置

```bash
# 抓橘子放绿碗
echo '{"object":"orange","container":"green bowl"}' | python run_skill.py pick_and_place

# 递送 / 扔垃圾 / 放桌面（container 特殊模式）
echo '{"object":"bottle","container":"person"}' | python run_skill.py pick_and_place
echo '{"object":"wrapper","container":"trash"}' | python run_skill.py pick_and_place
echo '{"object":"cup","container":"desk"}' | python run_skill.py pick_and_place

# 多类别 OR 检测
echo '{"object":"apple,orange,fruit","container":"red plate"}' | python run_skill.py pick_and_place

# 从用户手中接收物品
echo '{"container":"trash"}' | python run_skill.py fetch_from_user

# 双臂交接放入抽屉
echo '{"object":"orange","container":"drawer1"}' | python run_skill.py grasp_to_drawer
```

### 5.3 位姿与动作序列

```bash
# 列出已录制位姿
echo '{"command":"list","arm":"left"}' | python run_skill.py pose_execute
echo '{"command":"list","arm":"right"}' | python run_skill.py pose_execute

# 回放单个位姿
echo '{"command":"play","name":"home","arm":"left"}' | python run_skill.py pose_execute

# 回放位姿并指定速度
echo '{"command":"play","name":"grasp1","arm":"left","speed":50}' | python run_skill.py pose_execute

# 双臂并行
echo '{"command":"play","parallel":[{"arm":"left","name":"wave"},{"arm":"right","name":"open_gripper"}]}' | python run_skill.py pose_execute
```

### 5.4 抽屉操作（右臂轨迹回放）

`open_drawer` / `close_drawer` 是预录制的完整轨迹回放（不是简单位姿移动），强制右臂执行：

```bash
# 真机（SDK 直驱右臂 192.168.1.18:8080）
echo '{"command":"open_drawer"}' | python run_skill.py pose_execute
echo '{"command":"close_drawer"}' | python run_skill.py pose_execute

# 仿真（回放到 SimServer 右臂）
echo '{"command":"open_drawer"}' | SIM_MODE=1 python run_skill.py pose_execute
echo '{"command":"close_drawer"}' | SIM_MODE=1 python run_skill.py pose_execute
```

轨迹文件：`recorded_trajectories/right/{open,close}_drawer.json`

- `open_drawer`：home → 抓把手（夹爪 988→196）→ 拉开 → 松手（196→988）→ 回 home
- `close_drawer`：home → 推关 → 回 home（夹爪全程张开，不抓把手）

### 5.5 原子 Skill（独立 CLI）

```bash
echo '{"object":"orange"}' | python run_skill.py grasp
echo '{"object":"orange","container":"green bowl"}' | python run_skill.py place
python run_skill.py handover
python run_skill.py trash
python run_skill.py desk_place
```

---

## 6. 两种模式对比

| 维度 | 真机模式 | 仿真模式 |
|------|----------|----------|
| 启动脚本 | `bash start.bash`（6 标签） | `bash start_sim.bash`（2 标签） |
| 运行前缀 | 无 | `SIM_MODE=1` |
| 后端 | 真实机械臂 + Twin IK | PyBullet SimServer :8031 |
| 视觉/逆解 | 完整支持 | Phase 2（未路由） |
| 硬件要求 | 双臂 + 夹爪 + 相机 + GPU | 仅需 Python + ROS + PyBullet |
| 适用 | 实际作业 | 开发、演示、无硬件环境 |

---

## 7. 常见问题排查

### 7.1 仿真启动失败

**端口 8031 被占用**
```bash
pgrep -af sim_server.py      # 找到残留进程
kill <PID>                    # 或：bash start_sim.bash 会自动清理
```

**roscore 未就绪，SimServer 报 "Unable to register with master"**
- 用 `start_sim.bash` 启动会自动等 roscore 就绪（`until rostopic list` 轮询）。
- 手动启动时务必先起 roscore 再起 sim_server。

### 7.2 真机夹爪启动失败

```bash
ls /dev/ttyUSB*    # 检查串口装置号
# 右臂夹爪应为 /dev/ttyUSB0 (slave 9)，左臂为 /dev/ttyUSB1 (slave 1)
# 若装置号漂移，建议写 udev 规则按序列号锁定
```

### 7.3 YOLO-World 检测不到中文物体

YOLO-World 仅支持英文类名。中文名需先翻译（如 桃子→`peach`、瓶子→`bottle`、杯子→`cup`）。

### 7.4 AnyGrasp 导入失败

```bash
# 常见：`from gsnet import AnyGrasp` 导入失败，需在 conda 环境内执行
conda activate anygrasp
# 并确认 cuDNN 路径已 export（见 2.2）
```

### 7.5 仿真模式下机械臂位置漂移

占位桌面（`table.urdf`）会与机械臂基座碰撞、把臂顶出指令位姿。当前仿真已移除桌面（Phase 2 再按真实工作空间标定场景），若需加物体请确保不与臂基座重叠、高度匹配抓取位姿（~0.45m）。
