# Skill 提取与组合 Skill

openclaw 是**聪明的执行者**。这个 skill 描述**如何从代码库提取原子/二级 skill，组合成满足用户需求的多步流水线**。

> 这是**元 skill**——告诉你**怎么用其他 skill**，本身不是某个具体物理能力。

## 触发条件

调用本 skill 当：
- 用户提出任何与物理世界交互的需求
- 你需要决定"用哪个 skill / 怎么组合"
- `COMMANDS.md` 现成配方不完全匹配用户请求，需要自己拼装
- 用户问"你能做 X 吗"——先查字典再回答

---

## 核心理念：代码库即字典

你的能力**不在你自己脑子里，在代码库里**。先查字典，再说话。

### 字典分层

| 层级 | 文件 | 内容 |
|---|---|---|
| L0 入口 | `run_skill.py` | 所有 skill 的统一 CLI 入口（`python run_skill.py list` 看全部） |
| L1 命令清单 | `COMMANDS.md` | 官方推荐的 skill 调用配方（**第一手**，先查这里） |
| L2 架构上下文 | `CLAUDE.md` | skill 之间依赖关系、调用约束、协议细节 |
| L3 原子 skill 实现 | `skills/<name>.py` | 每个 skill 的 `run()` 方法签名 → 知道接受什么 kwargs |
| L4 二级 skill | `skills/pick_and_place.py`, `skills/fetch_from_user.py` | 已组合好的高级流水线 |
| L5 资源 | `recorded_poses.json`, `recorded_sequences/`, `robot_config.json` | 预录制位姿/序列/配置 |

### 查字典的标准顺序

收到用户请求后**按此顺序查**，不要凭记忆回答：

1. `grep -i <关键词> COMMANDS.md` — 看有没有现成配方
2. `python run_skill.py list` — 看有哪些 skill 注册了
3. 读候选 skill 的 `run()` 签名（`skills/<name>.py` 头部）— 知道接受什么 JSON 字段
4. 必要时读 `CLAUDE.md` 的"调用方式" / "特殊容器模式" / "开发原则"节
5. 涉及位姿时 `cat recorded_poses.json | python -c "import json,sys; print(list(json.load(sys.stdin).keys()))"` 看有哪些可用
6. 涉及双臂时 `ls recorded_sequences/` 看有没有现成 sequence

---

## 提取流程（四步）

### Step 1: 拆解需求

把用户的自然语言请求拆成**动作原语**：

| 用户说 | 拆解 |
|---|---|
| "把桃子放到盘子里" | detect(peach) → grasp(peach) → move(plate) → release — ⚠️ YOLO-World 只认英文类名 |
| "看看桌上有什么" | look_around → 报告结果 |
| "接着"（用户递物） | move(handover) → open_hand → wait → close_hand → place(dest) |
| "把这个从左手递给右手" | `pose_execute` 的 `parallel` 字段，或回放 `recorded_sequences/handover.json` |
| "把抽屉打开" | `pose_execute` 的 `command:"open_drawer"` |
| "把橘子放进抽屉" | `grasp_to_drawer`（4 阶段已实现） |
| "用右手递给我瓶子" | `pick_and_place` + `side:"right"` |

### Step 2: 匹配 skill

每个动作原语查字典找对应 skill：

| 动作原语 | 对应 skill |
|---|---|
| detect(X) + 报告 | `look_around` 或 `capture_at_handover` |
| grasp(X) + move(Y) + release | `pick_and_place`（4 模式：容器名 / `person` / `trash` / `desk`；可加 `side:right`） |
| 从用户手接物 | `fetch_from_user` |
| 抓物体放抽屉 | `grasp_to_drawer`（双臂 4 阶段流水线） |
| 开/关抽屉（单独） | `pose_execute` 的 `command:"open_drawer"/"close_drawer"` |
| 表演性动作 / 预录制 sequence | `pose_execute`（新 API：`command:"play"` + `name`/`arm`） |
| 双臂并行执行 | `pose_execute` 的 `parallel` 字段 |
| 双臂协同 sequence | `pose_execute` + `cat recorded_sequences/<name>.json \|` |

### Step 3: 组合调用

按 `CLAUDE.md` 的"开发原则"组合：

- **禁止跨 Skill 实例化**：高级 skill 内部用 `self.control_arm` / `self.control_hand` 内联，不创建新 Skill 实例
- **`run()` 必须先检查 kwargs 再回退 stdin**：JSON 从 stdin 或 kwargs 都能进
- **位姿查询统一用 `Config.get_pose()`**
- **shell pipeline 仅用于 stdin JSON 传递**，不要 `subprocess` 串多个 `run_skill.py`

### Step 4: 执行前自检

- conda 环境：`/home/zz/anaconda3/envs/anygrasp/bin/python`
- cuDNN 库路径：`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib`
- 4 端口在线：`lsof -ti:8000,8010,8011,8020`（应返回 4 PID）
- 当前 git 分支匹配（双臂功能需 `Double-arm-on-desk`）

---

## 已知 skill 速查

> **不要把这个表背下来**——`skills/` 目录随时在变。每次会话用 `python run_skill.py list` 实时查。此表只是入门向导。

### 高级（二级）skill

| skill | 干什么 | 输入 |
|---|---|---|
| `pick_and_place` | 抓取+放置（4 模式：容器/person/trash/desk；支持 `side:right`） | `{"object":"X","container":"Y"}` |
| `fetch_from_user` | 从用户手中接物+放置 | `{"container":"Y"}` |
| `grasp_to_drawer` | **双臂流水线**：左抓→右开抽屉→双臂交接→右放抽屉 | `{"object":"X","container":"drawer1"}` |
| `look_around` | 桌面环视 + GLM-4.5V 分析 | 无 |
| `capture_at_handover` | 拍用户手中物 | 无 |
| `pose_execute` | 位姿/序列/双臂并行/手势执行；含 `open_drawer`/`close_drawer` 直指令 | 见 `COMMANDS.md` |

### 原子 skill（一般不单独用，被高级 skill 内联）

| skill | 干什么 |
|---|---|
| `grasp` | 仅抓取 |
| `place` | 仅放置 |
| `handover` | 移到 handover 位 + 张手 |
| `trash` | 移到 trash 位 + 张手 |
| `desk_place` | 桌面随机放置 |

### 工具（非 skill）

| 工具 | 用途 |
|---|---|
| `tools/pose_record.py` | 录制新位姿到 `recorded_poses.json` |
| `tools/get_current_pose.py` | 读当前关节角度 |

---

## 组合配方（openclaw 自己沉淀的二级字典）

> 每次成功解决一个新需求，把配方记在这里。**这是 openclaw 持续变聪明的载体**——首次执行时是"探索"，记下来后下次就是"配方"。

<!-- 模板：
### 需求：<描述>

**用户原话**: "..."

**拆解**: primitive1 → primitive2 → ...

**实际调用**:
```bash
echo '{...}' | python run_skill.py <skill>
```

**注意**: ...

**首次成功**: YYYY-MM-DD
-->

### 需求：把物体从左手递给右手（双臂交接）

**用户原话**: "把这个从左手给右手" / "左手的东西放到右手上"

**拆解**: pose_sequence(handover_main 张手 → 用户放/左手抓 → handover_aux 接近 → 主手松 + 副手握)

**实际调用**:
```bash
cat recorded_sequences/handover.json | python run_skill.py pose_execute
```

**注意**: 双臂协同——主手先动，副手跟进。MEMORY 2026-07-05 实测可用。

**首次成功**: 2026-07-05

### 需求：把物体放进抽屉（双臂流水线）

**用户原话**: "把橘子放进抽屉" / "放到抽屉里"

**拆解**: 左臂抓取 → 右臂开抽屉 → 双臂交接（左→右）→ 右臂放入抽屉

**实际调用**:
```bash
echo '{"object":"orange","container":"drawer1"}' | python run_skill.py grasp_to_drawer
```

**注意**: 4 阶段流水线已完整实现（`skills/grasp_to_drawer.py`）。`open_drawer`/`close_drawer` 硬编码右臂。

**首次成功**: 2026-07-06（`drawer_cycle.json` 录制日期）

### 需求：单独开关抽屉（不抓物）

**用户原话**: "把抽屉打开" / "关上抽屉"

**拆解**: `pose_execute` 直指令

**实际调用**:
```bash
echo '{"command":"open_drawer"}'  | python run_skill.py pose_execute
echo '{"command":"close_drawer"}' | python run_skill.py pose_execute
```

**注意**: 只能右臂执行（硬编码）。

### 需求：用右臂执行抓放

**用户原话**: "用右手拿瓶子给我" / "右手扔垃圾桶"

**拆解**: `pick_and_place` + `side:"right"`

**实际调用**:
```bash
echo '{"object":"bottle","container":"person","side":"right"}' | python run_skill.py pick_and_place
```

### 需求：双臂并行执行不同动作

**用户原话**: "左手挥，右手张开"

**拆解**: `pose_execute` 的 `parallel` 字段

**实际调用**:
```bash
echo '{"command":"play","parallel":[
  {"name":"wave","arm":"left","speed":30},
  {"name":"open_gripper","arm":"right","speed":30}
]}' | python run_skill.py pose_execute
```

---

## 不预设能力边界

如果用户要的东西**不在"已知 skill 速查"里**：

1. **不要先说"做不到"** — 先按"查字典的标准顺序"查代码库
2. 读 `skills/` 下所有 `.py` 文件，看有没有可拼装的
3. 读 `tools/` 看有没有更底层的工具
4. 看预录制 pose 库 `recorded_poses.json` 和 `recorded_sequences/` 有没有相关位姿
5. 如果找到能拼的，执行（明确标注"探索性尝试，不保证成功率"）
6. 如果真没有，**诚实告知用户具体缺哪个 primitive**（不是笼统说"做不到"），并提议：
   - 用户用 `tools/pose_record.py` 录新 pose 后再拼
   - 或用户允许写新代码（需用户明准）

---

## 双臂协调执行原则

| 角色 | 硬件 | 任务 |
|---|---|---|
| 主手 | 左臂 RM75-B + Inspire 灵巧手（6 DoF） | 抓取、精细操作、递物 |
| 副手 | 右臂 RM75-B + 夹爪 | 支撑、固定、接收、辅助 |

**双臂顺序约束**：主手先动 → 副手跟进 → 主手释放。**不允许同时异动**（当前实现是序列化的）。

---

## 常见问题

| 问题 | 解决 |
|---|---|
| 用户说"做 X"但 X 不在 `COMMANDS.md` | 按"提取流程"四步走，不要直接拒绝 |
| 想用 `subprocess` 串多个 `run_skill.py` | 不推荐。优先用 kwargs 在同一进程传递；shell pipeline 仅用于 stdin JSON |
| 不知道某 skill 接受什么参数 | 读 `skills/<name>.py` 的 `run()` 方法签名 |
| 用户问"你能做 X 吗" | 不要直接答能/不能——**先查代码库再答** |
| 找不到现成 pose | 提议用户用 `tools/pose_record.py --arm left/right --name <name>` 录制 |
| 双臂 sequence 中左右臂位姿冲突 | dry-run 单步执行每个 pose，确认无碰撞再合 sequence |
