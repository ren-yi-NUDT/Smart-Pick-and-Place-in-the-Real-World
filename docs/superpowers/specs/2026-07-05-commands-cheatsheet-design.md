# Skill 调用速查表 (COMMANDS.md) 设计

**日期**: 2026-07-05
**分支**: Double-arm-on-desk
**作者**: Ren Yi + Claude

## 背景与目标

README.md 已包含完整架构说明、Skill 列表、通信协议、硬件信息（310 行），但命令分散在多个章节中，**查找具体可执行命令需要翻阅**。日常操作（特别是调试/演示时）需要一份「打开即用」的速查表。

**目标**：建立 `COMMANDS.md`，仓库根目录，与 README.md 平级，作为 Skill 调用的命令清单。**只列完整命令 + 中文注释，不做字段说明**。

## 范围

**包含**：
- `run_skill.py` 调用所有 skill 的完整命令
- 每条命令附中文行内注释说明用途
- 覆盖每个 skill 的常见使用场景（特殊 container 模式、多类别 OR、子命令变体等）

**不包含**（已由 README.md 覆盖）：
- 架构图、Skill-DB 设计哲学
- 通信协议、端口说明、Socket 协议细节
- 硬件型号、依赖列表
- Skill 内部实现说明
- 启动服务命令（`start.bash` 等）——README 已有

## 文件位置

`/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World/COMMANDS.md`

与 README.md 平级。文件名采用英文惯例（与 `LICENSE`、`CLAUDE.md` 一致）。

## 顶层结构

```
# Skill 调用速查表

> 前置：conda activate anygrasp + bash start.bash
> 所有命令在仓库根目录执行；JSON 通过 stdin 传入
> 架构/协议/硬件说明见 README.md

## 通用
## 抓取与放置
## 拍照与观测
## 位姿与动作序列
## 原子 Skill（独立 CLI）
```

## 每个 Skill 的呈现规则

- 标题：`### <skill_name> — <一句话说明>`
- 紧跟一个 bash 代码块，包含 3-6 条完整命令
- 每条命令上方一行中文注释，说明该命令的用途或差异点
- **不附字段说明表**——参数语义通过命令示例体现
- 特殊模式（如 `container: person/trash/desk`）作为 `pick_and_place` 的并列命令出现，不单列章节

## 各 Skill 命令覆盖清单

### 通用
- `python run_skill.py list` —— 列出所有已注册 skill

### 抓取与放置

#### `pick_and_place` — 检测→抓取→放置完整流程
- 抓橘子放碗里（基础用法）
- 递瓶子给用户（`container: person`）
- 扔糖纸进垃圾桶（`container: trash`）
- 放杯子到桌面（`container: desk`）
- 多类别 OR 检测（逗号分隔）

#### `fetch_from_user` — 从用户手中接收物品
- 接收物品放到粉色盘子
- 接收物品扔垃圾桶
- 接收物品放桌面

#### `grasp_to_drawer` — 双臂交接放入抽屉
- 左臂抓橘子 → 右臂放进抽屉

### 拍照与观测

#### `look_around` — 环视桌面 + VLM 分析
- 单次调用（无参数）

#### `capture_at_handover` — handover 位拍照识别
- 单次调用（无参数）

### 位姿与动作序列

#### `pose_execute` — 位姿/动作序列执行
实际只有 `play` 和 `list` 两个 `command` 子命令（录制/删除通过 `tools/pose_record.py`，不在 `run_skill.py` 内）：
- `play` + `name` —— 回放指定位姿
- `play` + `sequence` —— 执行 JSON 动作序列
- `play` + `parallel` —— 双臂并行
- `play` + `hand` —— 播放灵巧手手势
- `list` —— 列出已录制位姿

### 原子 Skill（独立 CLI 用）

> 加开场说明一句：高级 skill 内部已内联等效逻辑，原子 skill 仅独立 CLI 调用。

每个原子 skill 一节，1-3 条命令：
- `grasp` —— 视觉抓取
- `place` —— 视觉放置
- `handover` —— 递交给用户
- `trash` —— 扔垃圾
- `desk_place` —— 放桌面

## 长度预算

目标 100-130 行（含标题、空行、注释）。如果超出 150 行，回头精简冗余示例。

## 验证清单

完成后逐项核对：
- [ ] 每条命令可在 `conda activate anygrasp` + `start.bash` 起服务后直接执行
- [ ] JSON 格式合法（双引号、无尾逗号）
- [ ] skill 名与 `python run_skill.py list` 输出一致
- [ ] 注释中文，简短（一行）
- [ ] 顶层 5 个章节齐全
- [ ] 文件不超过 150 行

## 非目标 (Non-Goals)

- **不修改 README.md**：README 仍是架构/协议/硬件的权威说明；COMMANDS.md 仅是命令索引
- **不替代 CLAUDE.md**：CLAUDE.md 是 Claude 工作指导，COMMANDS.md 是用户操作速查
- **不收录启动/调试命令**：`start.bash`、`pkill` 模式、`lsof -ti:` 等留在 README 或 start.bash 注释
- **不写每 skill 的输出格式**：让用户运行后看实际输出
- **不写错误处理 / 故障排查**：故障排查归属 README 或独立文档
