# Skill 调用速查表

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

# 扔糖纸进垃圾桶
echo '{"object":"wrapper","container":"trash"}' | python run_skill.py pick_and_place

# 放杯子到桌面（随机预设位姿）
echo '{"object":"cup","container":"desk"}' | python run_skill.py pick_and_place

# 多类别 OR 检测（苹果/橘子/水果 任一命中即可）
echo '{"object":"apple,orange,fruit","container":"red plate"}' | python run_skill.py pick_and_place
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
```

## 原子 Skill（独立 CLI）

> 高级 skill 内部已内联等效逻辑，原子 skill 仅保留给独立 CLI 调用，避免在主流水线中建立额外 TCP 连接。

### grasp — 视觉抓取

```bash
# 视觉检测 + AnyGrasp 抓取规划 + Twin 轨迹执行
echo '{"object":"orange"}' | python run_skill.py grasp
```

### place — 视觉放置

```bash
# 检测容器位置 + 生成放置轨迹
echo '{"object":"orange","container":"green bowl"}' | python run_skill.py place
```

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
