# COMMANDS.md Skill 调用速查表 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在仓库根目录创建 `COMMANDS.md`，作为 `run_skill.py` 所有 skill 调用命令的可复制速查表（不含字段说明表，只列完整命令 + 中文注释）。

**Architecture:** 单文件、纯 Markdown，按使用场景分 5 个章节。每条命令上方一行中文注释；每个 skill 给 3-6 条覆盖常见场景的命令。文件头三行声明前置条件 + 链接到 README。

**Tech Stack:** Markdown only. No code generation, no build.

**Spec:** `docs/superpowers/specs/2026-07-05-commands-cheatsheet-design.md`

---

### Task 1: 创建文件骨架 + 通用 章节

**Files:**
- Create: `COMMANDS.md`

- [ ] **Step 1: 写入文件首部 + 通用章节**

用 Write 工具创建 `COMMANDS.md`，内容如下（一字不差）：

```markdown
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

（待 Task 2 填充）

## 拍照与观测

（待 Task 3 填充）

## 位姿与动作序列

（待 Task 3 填充）

## 原子 Skill（独立 CLI）

（待 Task 4 填充）
```

- [ ] **Step 2: 验证文件已创建且行数 < 30**

Run: `wc -l COMMANDS.md`
Expected: 输出 ≤ 30 行

- [ ] **Step 3: 提交**

```bash
git add COMMANDS.md
git commit -m "docs: scaffold COMMANDS.md with section headers"
```

---

### Task 2: 抓取与放置 章节

**Files:**
- Modify: `COMMANDS.md`

- [ ] **Step 1: 用 Edit 替换「（待 Task 2 填充）」占位符**

用 Edit 工具，将 `COMMANDS.md` 中的：

```
## 抓取与放置

（待 Task 2 填充）
```

替换为：

```markdown
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
```

- [ ] **Step 2: 验证 JSON 全部合法**

Run:
```bash
grep -oP "echo '\K\{[^}]+\}" COMMANDS.md | while read j; do echo "$j" | python -c "import json,sys; json.load(sys.stdin)"; done
```
Expected: 无输出（全部合法）。若有输出，定位修复。

- [ ] **Step 3: 提交**

```bash
git add COMMANDS.md
git commit -m "docs: add 抓取与放置 section to COMMANDS.md"
```

---

### Task 3: 拍照与观测 + 位姿与动作序列 章节

**Files:**
- Modify: `COMMANDS.md`

- [ ] **Step 1: 用 Edit 替换两个占位符**

第一个 Edit：将

```
## 拍照与观测

（待 Task 3 填充）
```

替换为：

```markdown
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
```

第二个 Edit：将

```
## 位姿与动作序列

（待 Task 3 填充）
```

替换为：

```markdown
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

# 执行 JSON 动作序列
echo '{"command":"play","sequence":"[{\"arm\":\"home\",\"hand\":\"open\",\"delay\":0.5}]","sequence_is_string":true}' | python run_skill.py pose_execute

# 双臂并行执行（每项可含 name/arm/speed/hand）
echo '{"command":"play","parallel":[{"arm":"left","name":"wave"},{"arm":"right","name":"open_gripper"}]}' | python run_skill.py pose_execute

# 播放灵巧手手势（预设：open/close/peace/grab/thumbs_up 等）
echo '{"command":"play","hand":"peace"}' | python run_skill.py pose_execute
```
```

- [ ] **Step 2: 验证 JSON 合法（应对转义后的引号）**

Run:
```bash
python -c "
import json
samples = [
    '{\"command\":\"list\",\"arm\":\"left\"}',
    '{\"command\":\"play\",\"name\":\"home\",\"arm\":\"left\"}',
    '{\"command\":\"play\",\"sequence\":\"[{\\\"arm\\\":\\\"home\\\",\\\"hand\\\":\\\"open\\\",\\\"delay\\\":0.5}]\",\"sequence_is_string\":true}',
]
for s in samples:
    json.loads(s)
print('all valid')
"
```
Expected: 输出 `all valid`

- [ ] **Step 3: 提交**

```bash
git add COMMANDS.md
git commit -m "docs: add 拍照与观测 and 位姿与动作序列 sections"
```

---

### Task 4: 原子 Skill 章节 + 最终验证

**Files:**
- Modify: `COMMANDS.md`

- [ ] **Step 1: 用 Edit 替换最后一个占位符**

将

```
## 原子 Skill（独立 CLI）

（待 Task 4 填充）
```

替换为：

```markdown
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
```

- [ ] **Step 2: 验证 skill 名与 `python run_skill.py list` 输出一致**

Run:
```bash
python run_skill.py list
```
Expected: 输出包含 `pick_and_place`, `fetch_from_user`, `grasp_to_drawer`, `look_around`, `capture_at_handover`, `pose_execute`, `grasp`, `place`, `handover`, `trash`, `desk_place`

如有不一致，修正 `COMMANDS.md` 中对应的 skill 名。

- [ ] **Step 3: 验证总行数在预算内**

Run: `wc -l COMMANDS.md`
Expected: 行数在 100-150 之间

若 > 150，回头精简冗余示例。

- [ ] **Step 4: 验证所有 JSON 命令可解析**

Run:
```bash
grep -oP "echo '\K\{[^}]+\}(?=' \|)" COMMANDS.md | while read j; do echo "$j" | python -c "import json,sys; json.load(sys.stdin)" || echo "BAD: $j"; done
```
Expected: 无 `BAD:` 输出

注意：`pose_execute` 中带转义引号的 JSON 可能不被这个简单 grep 匹配，已在 Task 3 Step 2 单独验证。

- [ ] **Step 5: 提交**

```bash
git add COMMANDS.md
git commit -m "docs: add 原子 Skill section, complete COMMANDS.md"
```

---

## 自检（Self-Review）

**Spec 覆盖**：
- ✅ 文件位置（仓库根目录 `COMMANDS.md`）
- ✅ 5 个章节（通用 / 抓取与放置 / 拍照与观测 / 位姿与动作序列 / 原子 Skill）
- ✅ 只列完整命令，无字段说明表
- ✅ 每条命令附中文行内注释
- ✅ 长度预算 100-150 行（验证步骤强制 ≤ 150）
- ✅ 不含启动/调试命令
- ✅ 不含字段说明表

**Placeholder 扫描**：每个占位符都有对应 Task 替换；最终文件无 `TBD`/`TODO`。

**类型一致性**：所有 skill 名通过 `python run_skill.py list` 校验。

**spec 修正**：spec 原「4 个 command 子命令」错写为 `play/record/list/delete`，实际 `pose_execute` 仅 `play` 和 `list`（已在 spec commit `75ee65f` 修正）。
