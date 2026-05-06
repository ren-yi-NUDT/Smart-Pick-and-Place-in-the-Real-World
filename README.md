# OpenClaw Workspace

具身智能大脑 CMLLR 的工作空间。OpenClaw 每次启动时加载此目录下的文件，构建 AI 的身份、记忆和技能。

## 目录结构

```
workspace/
├── IDENTITY.md                # AI 身份定义（名字、使命、性格）
├── SOUL.md                    # AI 行为准则和核心价值观
├── AGENTS.md                  # 会话启动流程、记忆规则、安全红线
├── USER.md                    # 用户信息与偏好（问候、挥手指令等）
├── MEMORY.md                  # 长期精选记忆（重要决策、项目状态、经验教训）
├── TOOLS.md                   # 硬件配置与数据获取快速参考
├── HEARTBEAT.md               # 心跳轮询任务（默认为空，跳过心跳）
│
├── skills/                    # 技能描述文件（SKILL.md 告诉 AI 如何执行）
│   ├── smart-grasp/           # 智能抓取放置（核心技能）
│   │   └── SKILL.md           # 调用方式、JSON 格式、5 种模式
│   ├── arm-gesture/           # 机械臂位姿执行与动作序列
│   │   └── SKILL.md           # 位姿列表、手势预设、序列格式
│   ├── self-improving/        # 自我改进与经验积累
│   ├── agent-browser/         # 浏览器代理
│   ├── find-skills/           # 技能发现
│   └── memory-setup/          # 记忆系统设置
│
├── memory/                    # 历史记忆存储
│   ├── logs/                  # 每日操作日志 (YYYY-MM-DD.md)
│   └── projects/              # 项目上下文
│
├── stuffs/                    # 辅助服务脚本
│   ├── feishu_bot_server.py   # 飞书机器人服务
│   └── ngrok                  # 内网穿透
│
├── state/                     # 运行时状态（空）
├── log/                       # 运行时日志（空）
└── .openclaw/                 # OpenClaw 内部配置
```

## 会话启动顺序

1. `SOUL.md` — 我是谁
2. `USER.md` — 用户是谁
3. `memory/logs/` — 近期日志
4. `skills/smart-grasp/SKILL.md` + `skills/arm-gesture/SKILL.md` — 物理世界交互手段
5. `MEMORY.md` — 仅主会话加载（含个人上下文，不在群聊中加载）

## 核心文件说明

### IDENTITY.md
AI 身份：名字 CMLLR，定位为具身智能大脑，专注于桌面级机械臂操控。

### SOUL.md
行为准则：不虚伪奉承、有独立观点、先自己尝试再求助、勇于承认做不到。

### AGENTS.md
最长的配置文件，定义了：
- 启动加载流程
- 记忆写入规则（事实→logs，决策→MEMORY，经验→self-improving）
- 安全红线（不泄露隐私、不执行破坏性命令）
- 群聊行为规范
- 心跳机制

### USER.md
用户画像与偏好，包括：
- 问候时操控机械臂挥手的指令
- 动作序列优化要求
- 工具调用时必须说明用途

### MEMORY.md
长期精选记忆，记录：
- 项目信息（Smart Pick and Place 架构）
- 硬件配置（机械臂、灵巧手、相机）
- 架构决策与经验教训
- 用户偏好

### TOOLS.md
硬件技术细节与数据获取命令：
- 机械臂 RM75-B 关节限位与 Socket 协议
- 灵巧手 Inspire Hand 手指控制数组
- RealSense D455 相机参数
- `run_skill.py look_around` / `run_skill.py capture_at_handover` 快速参考

### skills/ 目录
每个子目录的 `SKILL.md` 是 AI 执行该技能时的操作手册。关键的两个：
- **smart-grasp** — 智能抓取放置的完整调用指南（5 种模式、JSON 格式、前置条件）
- **arm-gesture** — 位姿执行与动作序列播放（位姿列表、手势预设、安全规则）

### memory/ 目录
持久化历史记录：
- `logs/YYYY-MM-DD.md` — 每日操作事实日志
- `projects/*.md` — 项目上下文

## 与代码库的关系

代码库在 `/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World`，workspace 不包含代码，只包含 **AI 的操作手册**。所有脚本调用命令写在 `skills/*/SKILL.md` 中，workspace 其他文件不重复写命令（避免不一致）。

## 飞书集成

- `stuffs/feishu_bot_server.py` — 飞书机器人后端
- `feishu_bot_config.md` — 飞书机器人配置
- `FEISHU_BOT_STATUS.md` / `OPENCLAW_FEISHU_STATUS.md` — 运行状态
- `stuffs/ngrok` — 内网穿透工具


claude --resume 1d42fb9f-51d4-40e9-8d6c-c3069c651ba0
