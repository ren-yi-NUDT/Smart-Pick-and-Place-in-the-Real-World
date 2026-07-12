# COMMANDS.md — Skill 调用速查表

> 这是 openclaw 的**命令字典**。每次用户提需求，先按"查字典的标准顺序"（见 `skill-extraction/SKILL.md`）查这里。
>
> 仅做参考！！！！！请完整查看 `skills/<name>.py` 代码逻辑！！！！

## 前置

```bash
conda activate anygrasp
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib
./start.bash  # 启动 ROS + Twin IK 服务（4 端口：8000/8010/8011/8020）
```

JSON 通过 stdin 传入；架构 / 协议 / 硬件说明见 `CLAUDE.md`。

## 通用

```bash
# 列出所有已注册 skill
python run_skill.py list
```

---

## 抓取与放置

### `pick_and_place` — 检测→抓取→放置完整流程

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
```

> ⚠️ **`object` 字段必须用英文类名**。YOLO-World（yolov8x-worldv2.pt）只识别英文开放词汇，
> 中文（如 `"桃子"`、`"瓶子"`）会得到 `no detections`。
> 把口语映射成英文：桃子→`peach`、橘子→`orange`、瓶子→`bottle`、苹果→`apple`、杯子→`cup`。
> `container` 名字同理（`"green bowl"` 不要写 `"绿碗"`）。

# 用右臂执行（默认 left，可显式指定 right）
echo '{"object":"bottle","container":"person","side":"right"}' | python run_skill.py pick_and_place

# 右臂扔垃圾
echo '{"object":"bottle","container":"trash","side":"right"}' | python run_skill.py pick_and_place
```

**container 取值**:

| 值 | 模式 | 行为 |
|---|---|---|
| 容器名（`"green bowl"`, `"pink plate"` 等） | 桌面放置 | YOLO 检测容器位置 → 放进去 |
| `"person"` | 人机递物 | 经中间路径点运动到 handover 位 → 张手 |
| `"trash"` / `"垃圾桶"` / `"garbage"` / `"bin"` | 扔垃圾 | 运动到扔垃圾位 → 张手 |
| `"desk"` / `"桌子"` / `"table"` | 放桌面 | 从 `desk_pose_1/2/3` 中随机选 → 张手 |

### `fetch_from_user` — 从用户手中接收物品

```bash
# 接收物品放到粉色盘子
echo '{"container":"pink plate"}' | python run_skill.py fetch_from_user

# 接收物品扔垃圾桶
echo '{"container":"trash"}' | python run_skill.py fetch_from_user

# 接收物品放桌面
echo '{"container":"desk"}' | python run_skill.py fetch_from_user
```

仅需 `container`，物品由用户递给机械臂。流程：移到 handover → 张手等待 → 用户放入 → 闭合 → 放置。

### `grasp_to_drawer` — 双臂交接放入抽屉 ⭐ 已实现

```bash
# 左臂抓橘子 → 右臂打开抽屉 → 双臂交接（左→右）→ 右臂放入抽屉
echo '{"object":"orange","container":"drawer1"}' | python run_skill.py grasp_to_drawer
```

**4 阶段流水线**:
1. 左臂（主手）抓取目标物体
2. 右臂（副手）打开抽屉（预录制位姿 `open_drawer`）
3. 双臂交接：左→右
4. 右臂放入抽屉（预录制位姿 `place_into_drawer`）

---

## 拍照与观测

### `look_around` — 环视桌面 + VLM 分析

```bash
# 循环 grasp1-4 拍照，调用 GLM-4.5V 分析场景
python run_skill.py look_around
```

返回桌面物品清单（名称、位置、颜色）。

### `capture_at_handover` — handover 位拍照识别

```bash
# 移动到 handover 位姿拍照，VLM 识别手中物品
python run_skill.py capture_at_handover
```

---

## 位姿与动作序列

### `pose_execute` — 位姿/动作序列执行（**新 API**）

> ⚠️ 字段名是 `name`（位姿名）+ `arm`（哪只臂），不是旧版的 `arm` 当位姿名用。

#### 列出位姿

```bash
echo '{"command":"list","arm":"left"}'  | python run_skill.py pose_execute
echo '{"command":"list","arm":"right"}' | python run_skill.py pose_execute
```

#### 回放单个位姿

```bash
echo '{"command":"play","name":"home","arm":"left"}' | python run_skill.py pose_execute
echo '{"command":"play","name":"grasp1","arm":"left","speed":50}' | python run_skill.py pose_execute
```

#### 执行 JSON 动作序列（序列化）

```bash
echo '{"command":"play","sequence":[
  {"name":"home","arm":"left","delay":0.5},
  {"name":"grasp-ready","arm":"left","delay":0.3},
  {"hand":"open","delay":0.3}
]}' | python run_skill.py pose_execute
```

#### 双臂并行执行 ⭐ 双臂协同

```bash
echo '{"command":"play","parallel":[
  {"name":"handover_pose","arm":"left","speed":30},
  {"name":"open_gripper","arm":"right","speed":30}
]}' | python run_skill.py pose_execute
```

#### 直接调用抽屉命令（右臂硬编码）

```bash
echo '{"command":"open_drawer"}'  | python run_skill.py pose_execute
echo '{"command":"close_drawer"}' | python run_skill.py pose_execute
```

> `open_drawer` / `close_drawer` 只能由右臂执行（硬编码 192.168.1.18）。

#### 播放灵巧手手势

```bash
echo '{"command":"play","hand":"peace"}' | python run_skill.py pose_execute
```

手势预设：`open` / `close` / `peace` / `rock` / `pointing` / `thumbs_up` / `ok` / `grab`

#### Sequence 字段

| 字段 | 说明 | 默认值 |
|---|---|---|
| **name** | 位姿名（必须存在于 `recorded_poses.json`） | 必填 |
| arm | `left` / `right` | `left` |
| hand | 手势预设名或 6 元素数组（仅左臂） | 可选 |
| delay | 延时秒数 | 0.5 |
| speed | 移动速度 0-100 | 30 |

#### 平台约束

- `ACTION_ARM_RESTRICTIONS`: 部分动作（`open_drawer`, `close_drawer`）硬编码指定臂，传入错的 `arm` 会被拒绝
- 序列开始前先进 `home`，结束后回 `home` 再进 `grasp-ready`
- 手势在 sequence 结束后恢复为 `open`
- 序列结束后相机不在桌面位，需调 `look_around` 或 `capture_at_handover`

---

## 原子 Skill（独立 CLI）

> 高级 skill 内部已内联等效逻辑，原子 skill 仅保留给独立 CLI 调用，避免在主流水线中建立额外 TCP 连接。

| skill | 干什么 | 输入 |
|---|---|---|
| `grasp` | 视觉抓取（YOLO + AnyGrasp + Twin 轨迹） | `{"object":"X"}` |
| `place` | 视觉放置 | `{"object":"X","container":"Y"}` |
| `handover` | 经中间点插值运动到 handover 位 + 张手 | 无 |
| `trash` | 移动到垃圾桶位 + 张手 | 无 |
| `desk_place` | 从 `desk_pose_1/2/3` 中随机选 + 张手 | 无 |

---

## 工具（非 skill）

| 工具 | 用途 |
|---|---|
| `tools/pose_record.py record --name X --arm left/right` | 录制新位姿到 `recorded_poses.json` |
| `tools/pose_record.py list` | 列出已录制位姿 |
| `tools/pose_record.py delete --name X` | 删除位姿 |
| `tools/get_current_pose.py` | 读当前关节角度 |

---

## 已录制的关键 sequence

| 文件 | 用途 |
|---|---|
| `recorded_sequences/drawer_cycle.json` | 右臂开关抽屉完整 cycle（home → approach → grasp_handle → drawer_open → drawer_close → release → home） |
| `recorded_sequences/handover.json` | 双臂交接 4 步序列（实测可用，MEMORY 2026-07-05） |

回放：
```bash
cat recorded_sequences/<name>.json | python run_skill.py pose_execute
```

---

## 用户意图映射（口语 → container 值）

| 用户说 | container |
|---|---|
| "放桌上" / "放桌子上" / "放桌子" | `"desk"` |
| "给我" / "递给我" / "我要" | `"person"` |
| "扔掉" / "扔垃圾桶" / "丢掉" | `"trash"` |
| "放抽屉里" / "放进抽屉" | `"drawer1"` + `grasp_to_drawer` skill |
