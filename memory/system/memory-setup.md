# Memory Setup 系统配置

> 此文件记录 memory-setup 技能的使用方法，确保每次会话都能正确使用记忆系统

## 核心原则

**不要用脑子记，写进文件！** 📝

## 目录结构

```
workspace/
├── MEMORY.md              # 长期精选记忆
│   - 重要决策
│   - 偏好设置
│   - 活跃项目
│   - 经验教训
│
└── memory/
    ├── logs/              # 每日日志
    │   └── YYYY-MM-DD.md  # 按日期命名
    │
    ├── projects/          # 项目上下文
    │   └── project-name.md
    │
    ├── groups/            # 群聊上下文
    │   └── group-name.md
    │
    └── system/            # 系统配置
        └── memory-setup.md (本文件)
```

## 写入规则速查

| 内容类型 | 写入位置 | 示例 |
|----------|----------|------|
| 今天发生了什么 | `memory/logs/YYYY-MM-DD.md` | 完成了抓取测试 |
| 重要决策/偏好 | `MEMORY.md` | 机械臂速度≤5 |
| 项目相关上下文 | `memory/projects/*.md` | 项目启动命令 |
| 可复用工作流 | `~/self-improving/` | 调试流程 |
| 群聊信息 | `memory/groups/*.md` | 群成员、话题 |
| 硬件配置 | `TOOLS.md` | IP、端口、参数 |

## 每日日志模板

```markdown
# YYYY-MM-DD — 每日日志

## 上午
- [时间] 事件描述
- 决策: xxx
- 待办: xxx

## 下午
- [时间] 事件描述

## 今日总结
- 完成了 xxx
- 学到了 xxx
- 明天要 xxx
```

## 记忆召回

在回答关于过去的问题前，使用：
```bash
memory_search "查询关键词"
memory_get "path" --from N --lines M
```

## 重要提醒

1. **会话结束后记忆消失** — 必须写入文件才能持久化
2. **MEMORY.md 是精选** — 不是所有东西都放进去
3. **每日日志是原始记录** — 详细但不一定重要
4. **定期整理** — 把日志中的重要内容提炼到 MEMORY.md

---

*Created: 2026-03-09*
