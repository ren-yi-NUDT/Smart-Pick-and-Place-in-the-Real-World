# 全闭环 PyBullet 虚拟仿真（调试用）设计

日期：2026-08-17
状态：待评审

## 1. 目标

在不连接真实硬件的情况下，用项目已有的 PyBullet 数字孪生（`dependence/twin_inference/`）跑通 skill 流水线（`pick_and_place`、`fetch_from_user` 等）的「检测 → 抓取 → 放置」全链路，**用于调试**。

核心诉求：把机械臂运动、夹爪开合、RGB-D 相机三类硬件信号全部路由到仿真，让整个流水线在仿真里闭环，无需真实机械臂、夹爪、RealSense 相机、ROS bringup。

## 2. 范围

**做**：
- 新增 `sim_mode` 开关，把 `arm` / `hand` / `camera` 三个后端切换为仿真客户端。
- 新增 SimServer 进程（PyBullet），加载 `dual_arm.urdf` + 场景（桌面 + 可抓物体 + 容器），提供关节轨迹/位姿执行、夹爪开合、RGB-D 渲染、关节角回报。
- 仿真相机内参与 `core/transforms.py` 对齐，使 YOLO 检测 + AnyGrasp 抓取 + 放置定位能在合成图像上正确投影。
- 在 `sim_mode` 下用 SimServer 的真实位姿旁路 ROS TF 变换。
- `start.bash` 增加 SimServer 启动项（替代 ROS bringup + 夹爪服务）。

**不做**（本期）：
- 外部 RoboTwin / Isaac Sim 框架。
- 真实传感器噪声 / 域随机化。
- 双臂协同 handover 的完整物理仿真（阶段二）。
- 训练策略模型。

## 3. 架构

```
skill 流水线 (pick_and_place / fetch_from_user ...)
   │ self.arm           self.hand            self.camera        self.twin
   ▼                    ▼                    ▼                  ▼
[sim_mode 开关]   SimArmClient       SimGripperClient    SimCamera     (twin 保持不变)
   │                    │                    │
   └──────────┬─────────┴────────────────────┘
              ▼
   SimServer (PyBullet 进程, 端口 8031)
     加载 dual_arm.urdf + 场景
     命令：execute_trajectory / move_to_pose / gripper / get_rgbd / get_joint_state / reset
```

- **twin 轨迹生成保持不变**：`twin.py`（端口 8020/8021）已能在仿真里做 IK/碰撞/轨迹生成，返回关节轨迹（弧度）。SimServer 只负责「执行」与「相机」。
- 两条 PyBullet 进程并存：`twin.py`（单臂 URDF，用于 IK）+ `sim_server.py`（双臂 URDF，用于执行/相机）。二者都从同一组 URDF/mesh 建模，关节名/限位一致，轨迹可互相传递。

## 4. 组件

### 4.1 SimServer（新：`dependence/twin_inference/sim_server.py`）

基于现有 `sim_world.World` + `robot.ErdaijiRobot` 改造。JSON socket 服务，端口 8031，协议：4 字节大端长度前缀 + JSON（与 twin 响应一致）。

命令集（`{"cmd": ..., ...}`）：

| cmd | 入参 | 行为 |
|---|---|---|
| `reset` | — | 复位双臂到默认位姿、夹爪张开、场景复位 |
| `execute_trajectory` | `side`, `trajectory`(关节角列表，**度**), `speed` | 逐步驱动关节，动画执行并逐步 step |
| `move_to_pose` | `side`, `pose`(关节角 dict，**度**) | 移动到指定关节位姿 |
| `gripper` | `side`, `action`(`open`/`close`), `value`(0..1000) | 驱动夹爪手指关节 |
| `get_rgbd` | `side` | 渲染该臂相机链接处的 RGB + depth(uint16 mm) |
| `get_joint_state` | `side` | 返回当前关节角（度） |

关键点：
- **关节单位约定**：真实 `ArmClient` 接口**全程用度**——`move_to_named_pose(pose_dict)`（来自 `robot_config.json`，度）与 `execute_trajectory(trajectory)`（twin 返回弧度，skill 已 `/np.pi*180` 转成度后才传入，见 `grasp.py:197`）都是度。SimArmClient 必须与真实 `ArmClient` 保持一致（度进）。SimServer 内部把度转弧度喂 PyBullet（`p.resetJointState`/`setJointMotorControl2`/`ErdaijiRobot.move_joint` 均用弧度）。
- **相机渲染**：仓库目前没有 Camera 类（`from camera import Camera` 被注释、`set_up_camera` 未启用）。需用 `p.getCameraImage` 从零实现：根据臂的相机 link 位姿（`cam_link_grasp` 及左/右臂各自相机链接）计算 `viewMatrix`/`projectionMatrix`，渲染 640×480 RGB + depth buffer，depth 转为 uint16 mm（与 `RealSenseCapture` 对齐，供 `pixel_to_camera_point` 直接使用）。
- **夹爪 mimic 关节**：PyBullet 不支持 URDF `<mimic>` 标签。`dual_arm.urdf` 中左/右夹爪各有一个主动关节（`L_finger_joint` / `R_finger_joint`，revolute）+ 若干 `mimic` 子关节（乘数 ±1）。需在 SimServer 里维护主动关节与 mimic 子关节的映射，开合时按乘数同时驱动全部手指关节（`p.setJointMotorControl2` position control）。夹爪开合值映射：Robotiq 85 的 `[v,v]`（0=闭合,1000=张开）→ 手指关节角度（需标定一个线性/查表映射，MVP 用近似线性即可）。
- **场景**：`p.loadURDF("plane.urdf")` + 桌面（一个 box）+ 若干可抓物体（`p.loadURDF` 原生 box/sphere/cylinder，带纯色或纹理，使 YOLO-World 能检出 `cup`/`bowl`/`box` 等类名）+ 容器（`bowl`/`box`）。物体位置与真实工作空间大致对应。

### 4.2 仿真客户端（新：`core/sim_arm.py` / `core/sim_gripper.py` / `core/sim_camera.py`）

接口对齐真实客户端，便于 `base.py`/`arm_side.py` 透明替换：

- `SimArmClient`：`connect()`、`close()`、`move_to_named_pose(pose_dict, speed)`、`execute_trajectory(trajectory, speed)`、`reset_cmd`/`start_cmd`/`add_js_cmd`/`send_cmds`（后四个可空实现或直接映射到 execute_trajectory）。内部走 socket 到 SimServer。
- `SimGripperClient`：`open()`、`close()`、`get_state()`、`is_grasping()`、`is_fully_open()`、`get_finger_deviation()`。`is_grasping()` 在 MVP 里按「夹爪目标值小于阈值即视为抓握」简化返回（不依赖真实 gOBJ 标志），或由 SimServer 根据物体是否在手指间做近似判断（阶段二）。
- `SimCamera`：`get_rgbd()` → 返回 (H,W,3) uint8 RGB + (H,W) uint16 depth，与 `RealSenseCapture` 完全同构；`close()` 空实现。构造参数兼容 `RealSenseCapture(width,height,fps,save_path,serial)`。

### 4.3 配置与分发（改：`core/config.py`、`skills/base.py`、`core/arm_side.py`）

- 开关：`SIM_MODE=1` 环境变量，或 `robot_config.json` 的 `shared.sim_mode: true`（两者任一为真即启用；环境变量优先，便于不改配置文件临时切换）。
- `config.py`：新增 `Config.sim_mode` 属性。
- `skills/base.py`：
  - `Skill.arm` property：`sim_mode` 时返回 `SimArmClient`。
  - `Skill.hand` property：`sim_mode` 时返回 `SimGripperClient`。
  - `Skill.camera` / `_make_camera`：`sim_mode` 时返回 `SimCamera`。
- `core/arm_side.py`：`ArmSide.arm` / `ArmSide.hand` property 在 `sim_mode` 时返回对应 Sim 客户端（host 指向 SimServer 端口 8031）。

### 4.4 启动（改：`start.bash` + 新增脚本）

- `start.bash` 增加一个 SimServer tab：`python3 sim_server.py`（conda anygrasp + source ROS，因为 `sim_world.py` 依赖 `rospy`）。
- 提供 `start_sim.bash`（或 `start.bash --sim`）作为仿真模式专用启动：只启动 SimServer + twin left/right + AnyGrasp Server（**不启动** ROS bringup、真实夹爪服务）。

## 5. 关键集成点（风险最高处，必须正确）

1. **相机内参对齐**：`core/transforms.py` 中左相机 `fx=fy=392.268, cx=325.468, cy=242.282`（右相机 fx=385.978, fy=385.347, cx=318.222, cy=238.816），图像 640×480。SimCamera 的 `projectionMatrix` 必须用这些内参反推（`fx/fy/cx/cy → PyBullet projectionMatrix`），否则 `graspcam2pixel` / `pixel_to_camera_point2` 投影会系统性偏移。
2. **TF 旁路**：真实流水线里 `Skill.save_current_transformation()` 从 ROS TF 拿 `base→camera` 与 `hand_effector→arm_endlink` 变换。仿真里无 ROS TF。需在 `sim_mode` 下让 `save_current_transformation` 改从 SimServer 拿相机 link / 末端 link 的世界位姿，计算同样的 4×4 变换（`transform_world_to_camera` 依赖 `T_base_to_cam`）。这是全闭环能否闭环的关键。
3. **AnyGrasp 在合成深度图上的效果**：AnyGrasp 由真实传感器数据训练，合成渲染的 depth（无噪声、边缘硬）可能抓取质量差甚至为空。对「调试流水线控制流」不影响；若要验证抓得准，需加**真值抓取兜底**（用物体已知位姿直接生成抓取姿态，绕过 AnyGrasp）。此项作为可选扩展，MVP 先跑通 YOLO 检测 + 放置定位，抓取质量后续调。
4. **关节单位**：见 4.1，`execute_trajectory` 与 `move_to_named_pose` **都是度**，必须和真实 `ArmClient` 一致；SimServer 内部转弧度喂 PyBullet。
5. **夹爪 mimic**：见 4.1，必须按乘数驱动全部手指关节，否则手指散架。

## 6. 数据流（以 `pick_and_place` 为例）

1. `run_skill.py` 读 JSON → `PickAndPlaceSkill.run()`（`SIM_MODE=1`）。
2. 抓取阶段：`self.camera.get_rgbd()` → SimCamera → SimServer 渲染 RGB-D。
3. `perception.detect_objects(rgb)` → YOLO-World 检测目标类。
4. `perception.detect_grasps(rgb, depth)` → AnyGrasp（8030，保持真实进程）→ 候选抓取。
5. `twin.generate_trajectory2(...)` → twin.py（8020）→ 关节轨迹（弧度）。
6. `self.arm.execute_trajectory(trajectory)` → SimArmClient → SimServer 动画执行。
7. `self.hand.close()` → SimGripperClient → SimServer 夹爪闭合。
8. 放置阶段同理，`move_to_named_pose` / `execute_trajectory` + `hand.open()`。
9. 全程关节角/相机位姿来自 SimServer，TF 变换走 5.2 的旁路。

## 7. 分阶段

- **阶段一（MVP，先跑通左臂单臂）**：SimServer 支持左臂执行 + 左夹爪 + 左相机渲染；`sim_mode` 切换三个客户端；TF 旁路；`pick_and_place` 全链路冒烟跑通（哪怕抓取质量粗糙）。
- **阶段二**：双臂（右臂同左臂）、夹爪 grasp 判定近似、真值抓取兜底、场景细化（多物体/容器/杂波）、`fetch_from_user` 与其它 skill 验证。

## 8. 测试策略

- **单元**：`SimArmClient`/`SimGripperClient`/`SimCamera` 对 mock SimServer 的协议收发；`Config.sim_mode` 开关解析；关节单位转换；夹爪 mimic 映射；相机内参→projectionMatrix 的数值正确性。
- **集成（人工，GUI）**：启动 SimServer（GUI 模式），跑 `pick_and_place`，观察双臂/夹爪动画、RGB-D 渲染质量、检测/放置结果。
- **回归**：真实模式（`SIM_MODE` 未设）下现有 49 项测试与 `run_skill.py list` 行为不变。

## 9. 已知约束 / 待定

- `sim_world.py`/`robot.py` 依赖 `rospy`，SimServer 需在 ROS 环境（source /opt/ros/noetic）下启动，即使不跑 ROS bringup。
- AnyGrasp Server（8030）在仿真模式下仍需启动（真实进程，加载模型）。是否替换为真值抓取兜底待阶段二评估。
- 左臂「抓取仍存在部分偏移待修正」（见当前分支提交信息）——该问题是真实臂与孪生模型不一致导致，仿真里可能表现不同，调试时需注意区分「流水线 bug」与「孪生模型偏差」。
