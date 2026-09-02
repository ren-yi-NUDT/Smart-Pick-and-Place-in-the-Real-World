# Skill 调用速查表

仅做参考！！！！！请完整查看代码逻辑！！！

> 前置：`conda activate anygrasp` + `bash start.bash`（启动 ROS + Twin IK 服务）
> 所有命令在仓库根目录执行；JSON 通过 stdin 传入
> 架构 / 协议 / 硬件说明见 [README.md](README.md)

## 通用

```bash
# 列出所有已注册 skill
python run_skill.py list
```

## 抓取与放置

### pick_and_place — 检测→抓取→放置完整流程

```bash
# 抓橘子放碗里
echo '{"object":"orange","container":"green bowl"}' | python run_skill.py pick_and_place

# 递瓶子给用户（人机递物）
echo '{"object":"bottle","container":"person"}' | python run_skill.py pick_and_place

# 扔瓶子进垃圾桶
echo '{"object":"bottle","container":"trash"}' | python run_skill.py pick_and_place

# 放杯子到桌面（随机预设位姿）
echo '{"object":"cup","container":"desk"}' | python run_skill.py pick_and_place

# 多类别 OR 检测（苹果/橘子/水果 任一命中即可）
echo '{"object":"apple,orange,fruit","container":"red plate"}' | python run_skill.py pick_and_place

# 用右臂执行（默认 left，可显式指定 right）
echo '{"object":"bottle","container":"person","side":"right"}' | python run_skill.py pick_and_place

# 右臂扔垃圾
echo '{"object":"bottle","container":"trash","side":"right"}' | python run_skill.py pick_and_place
```

### fetch_from_user — 从用户手中接收物品

```bash
# 接收物品放到粉色盘子
echo '{"container":"pink plate"}' | python run_skill.py fetch_from_user

# 接收物品扔垃圾桶
echo '{"container":"trash"}' | python run_skill.py fetch_from_user

# 接收物品放桌面
echo '{"container":"desk"}' | python run_skill.py fetch_from_user
```

### grasp_to_drawer — 双臂交接放入抽屉

```bash
# 左臂抓橘子 → 右臂打开抽屉 → 双臂交接 → 右臂放入抽屉
echo '{"object":"orange","container":"drawer1"}' | python run_skill.py grasp_to_drawer
```

## 拍照与观测

### 水果/蔬菜分拣

默认任务配置已经是水果/蔬菜分拣：水果放入粉色盘子，蔬菜放入蓝色盘子。
建议先只规划，检查日志中的场景识别、目标盘子和轨迹；确认无误后再执行。
执行时先由双相机 VLM 建立物体分类和左右臂任务队列；之后每个任务调用对应机械臂的单臂视觉抓取，由该臂自己的相机重新生成抓取位姿。采用 home 同步流水线：第一只臂完成单臂视觉抓取并回 home，第二只臂才开始抓取；第二只臂回 home 后，第一只臂放置，再放置第二只臂。
全部动作成功后，双臂再次回到 home，并确认左右夹爪完全打开。

```bash
# 真机：只拍照、识别并规划，不抓取
python -m tools.dual_vlm_sorting --plan-only

# 真机：确认工作区安全且 plan.json 正确后执行
DUAL_SORT_REAL_CONFIRM=1 python -m tools.dual_vlm_sorting --execute --real-confirm

# 仿真：先规划，再执行
SIM_MODE=1 python -m tools.dual_vlm_sorting --sim --plan-only --yes
SIM_MODE=1 python -m tools.dual_vlm_sorting --sim --execute --yes
```

### 双臂 Charuco 多点标定

标定脚本需要 `cv2.aruco`，请使用 `anygrasp` 环境。先打印并实测 24 mm 方格的 Charuco 板，保持两臂相机固定，在至少 8 个不同位置采集；保存前脚本会校验矩阵并备份旧的 `robot_config.json`。

```bash
# 生成/更新打印板
/home/zz/anaconda3/envs/anygrasp/bin/python tools/calibrate_arms.py generate-charuco

# 交互式采集（双臂固定，标定板每次换位置且同时可见）
/home/zz/anaconda3/envs/anygrasp/bin/python tools/calibrate_arms.py calibrate

# 使用未参与拟合的新位置做至少 3 点验证
/home/zz/anaconda3/envs/anygrasp/bin/python tools/calibrate_arms.py verify
```

### look_around — 环视桌面 + VLM 分析

```bash
# 循环 grasp1-4 拍照，调用 GLM-4.5V 分析场景
python run_skill.py look_around
```

### capture_at_handover — handover 位拍照识别

```bash
# 移动到 handover 位姿拍照，VLM 识别手中物品
python run_skill.py capture_at_handover
```

## 位姿与动作序列

### pose_execute — 位姿/动作序列执行

```bash
# 列出左臂所有已录制位姿
echo '{"command":"list","arm":"left"}' | python run_skill.py pose_execute

# 列出右臂所有已录制位姿
echo '{"command":"list","arm":"right"}' | python run_skill.py pose_execute

# 回放单个位姿
echo '{"command":"play","name":"home","arm":"left"}' | python run_skill.py pose_execute

# 回放位姿并指定速度（0-100）
echo '{"command":"play","name":"grasp1","arm":"left","speed":50}' | python run_skill.py pose_execute

# 执行 JSON 动作序列（每步可含 name/arm/hand/speed/delay）
echo '{"command":"play","sequence":[{"name":"home","arm":"left"},{"hand":"open","delay":0.5}]}' | python run_skill.py pose_execute

# 双臂并行执行（每项可含 name/arm/speed/hand）
echo '{"command":"play","parallel":[{"arm":"left","name":"wave"},{"arm":"right","name":"open_gripper"}]}' | python run_skill.py pose_execute

# 播放灵巧手手势（预设：open/close/peace/grab/thumbs_up 等）
echo '{"command":"play","hand":"peace"}' | python run_skill.py pose_execute

# 控制左/右 Robotiq 夹爪（未指定 arm 时默认为 left）
echo '{"command":"play","hand":"open","arm":"left"}' | python run_skill.py pose_execute
echo '{"command":"play","hand":"open","arm":"right"}' | python run_skill.py pose_execute
```

#### 抽屉操作（右臂轨迹回放）

`open_drawer` / `close_drawer` 是预录制的完整轨迹回放（SDK 直驱右臂 192.168.1.18:8080，默认 1.5x 速度），**不是简单的位姿移动**。强制右臂执行，`arm` 参数可省略。

```bash
# 开抽屉：home → 抓把手 → 拉开 → 松手 → 回 home
echo '{"command":"play","name":"open_drawer"}' | python run_skill.py pose_execute

# 关抽屉：home → 推关 → 回 home（不抓把手）
echo '{"command":"play","name":"close_drawer"}' | python run_skill.py pose_execute
```

轨迹文件：`recorded_trajectories/right/{open,close}_drawer.json`

## 原子 Skill（独立 CLI）

> 高级 skill 内部已内联等效逻辑，原子 skill 仅保留给独立 CLI 调用，避免在主流水线中建立额外 TCP 连接。

### grasp — 视觉抓取

```bash
# 视觉检测 + AnyGrasp 抓取规划 + Twin 轨迹执行
echo '{"object":"orange"}' | python run_skill.py grasp
```

### place — 视觉放置

```bash
# VLM框/容器内部安全区域/多SE(3)候选/Twin筛选/近距离视觉修正/释放后验证
echo '{"object":"orange","container":"green bowl"}' | python run_skill.py place
# 可选：指定右臂和橘子保守尺寸（米）
echo '{"object":"orange","container":"pink bowl","side":"right","object_size_m":0.06}' | python run_skill.py place
```

`object` 用于释放后的“物体是否进入容器”验证；省略时仍会检查夹爪是否打开和容器是否可见，但属于兼容模式，不能做物体语义确认。

### handover — 递交给用户

```bash
# 插值轨迹经中间点运动到 handover 位姿
python run_skill.py handover
```

### trash — 扔垃圾

```bash
# 移动到垃圾桶位姿松手
python run_skill.py trash
```

### desk_place — 放桌面

```bash
# 从 desk_pose_1/2/3 中随机选一个执行
python run_skill.py desk_place
```
