# 双臂交接 sim 后端设计

日期：2026-09-02
状态：已获用户批准的设计，待实施

## 背景与目标

`core/dual_handover.py` 的 `play()` 是多进程 SDK 直驱实现（CAN-FD 10ms 透传 + 定时夹爪事件），
无 sim 实现，导致 sim 模式下 `grasp_to_drawer`（阶段2）、`pick_and_place` 跨臂放置、
`dual_handover` skill、`tools/play_dual_handover.py` 全部失败。

目标：给 `play()` 增加 sim 后端，使 sim 模式下双臂交接可以真实回放（两臂同一时刻
处于各自录制轨迹的对应位置），与"仿真模式是基石"的项目原则一致。

范围限定（用户确认）：

- 只做 dual_handover 的 sim 端，不做录制/回放大合并
- SimServer 要求**真同时**：新增 `execute_dual_trajectory` 命令，同一物理循环同步推进两臂
  （SimServer 现有全局锁使两个并发连接串行，无法靠多线程连接实现同时性）
- 真机侧多进程 SDK 直驱实现不动（CAN-FD 实时性依赖，:8010 socket 路由无法替代）

## 现状事实（设计依据）

- 轨迹格式 `timed_dual_arm_handover`：`waypoints = [{elapsed_ms, left[7], right[7]}, ...]` +
  `events = [{time_s, event: right_close/left_open/both_home}]`
- 左右方向共用同一录制文件；`direction` 只决定事件角色（`left_to_right`：right_close 接、
  left_open 放；`right_to_left` 角色互换，时刻不变）
- SimServer 执行模型：`execute_trajectory` 逐航点速度封顶（`ARM_MAX_VEL`）位置控制、
  物理步进推进、**不保真实墙钟时长**；全程持 `self._lock`；夹爪命令带吸附检测
  （`object_detected`）
- 既有先例：`pose_execute._play_trajectory_sim` 单臂轨迹 sim 回放 = 分段
  `execute_trajectory` + 段间夹爪事件，sim 保相对顺序、不保绝对时长

## 设计

### 数据流

```
play() (core/dual_handover.py)
 ├─ 真机：现有多进程 SDK 直驱（不动）
 └─ Config.sim_mode == True → _play_sim()
       ├─ 复用 _load() 校验 + direction 角色互换逻辑（同一份轨迹文件）
       ├─ _segment() 按事件时刻把 paired waypoints 切段
       ├─ 每段：SimArmClient.execute_dual_trajectory(left_seg, right_seg)
       └─ 段间：SimGripperClient 触发 close_event / open_event（角色按 direction 换）
```

### 1. SimServer 新命令（`dependence/twin_inference/sim_server.py`，约 40 行）

- `dispatch` 新增分支 `execute_dual_trajectory`
- 请求：`{"cmd": "execute_dual_trajectory", "left": [[7 弧度值], ...], "right": [...]}`
  （两组等长）
- 新私有方法 `_move_joints_smooth_dual(l_target, r_target)`：两臂各自 `move_joint`
  后共同步进物理直到都到位，`ARM_MAX_VEL` 封顶，与单臂版 `_move_joints_smooth` 行为一致
- `_execute_dual_trajectory(req)`：持 `self._lock` 一次，zip(left, right) 逐对调用
  dual 平滑推进
- 校验：轨迹为空或左右长度不等 → `{"value": False, "info": {"error": ...}}`

### 2. SimArmClient（`core/sim_arm.py`，+3 行）

新增 `execute_dual_trajectory(left, right)` 方法，复用既有 `_send` 长度前缀协议，
风格与 `execute_trajectory` 一致。

### 3. dual_handover sim 后端（`core/dual_handover.py`）

- `play()` 开头：`Config.sim_mode` 为真时走 `_play_sim(name, speed, direction)`
  （与 `pose_execute._play_trajectory` 的真机/sim 双分支同模式）
- `_load()` 返回角度制；`_play_sim` 在下发前逐航点转弧度（deg→rad 责任在客户端，
  与 `SimArmClient.execute_trajectory` 现有约定一致）
- `_segment(trajectory_left, trajectory_right, event_times, speed)` 纯函数：
  按事件时刻（`time_s / speed`）切段，返回 `[(left_seg, right_seg, pending_event), ...]`，
  事件落在段边界上；`both_home` 忽略（轨迹本身含回 home 段，与真机回放一致）；
  无事件退化为单段，不特判
- 夹爪事件经 `SimGripperClient`（:8031，带 `object_detected` 反馈），角色映射复用
  `direction` 逻辑：`left_to_right` → 段界触发 right close、left open；反向互换
- 速度倍率只影响事件切分时刻；轨迹推进由物理步进决定（与既有 sim 语义一致）
- `require_confirmation` 在 sim 下忽略

### 4. 错误处理

- SimServer 返回 `value: False` → 客户端 raise `RuntimeError` → `play()` 现有
  try/except 路径返回 False，调用方（grasp_to_drawer 等）行为不变
- SimGripperClient 连接失败 → 同上抛错路径
- sim 下不做轨迹终点确认（真机的 `_at_target` 等待是 SDK 语义，sim 的
  `execute_*` 返回即到位）

### 5. 测试

- pytest（无需 PyBullet）：`_segment` 切段边界、事件角色映射（两个 direction）、
  空事件退化为单段、speed 缩放
- 手动验证：`start_sim.bash` 后跑 `grasp_to_drawer` 全流程，确认阶段2 两臂同步运动、
  t≈8s 右夹爪闭合、t≈10s 左夹爪打开、阶段3-5 继续

## 明确不做

- 录制端（`tools/record_dual_handover.py`、`tools/pose_record.py`）不动
- `pose_execute` / 单臂轨迹不迁移到统一引擎
- 三套录制格式不合并不迁移
