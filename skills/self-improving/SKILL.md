---
name: Self-Improving Agent (Proactive Self-Reflection)
slug: self-improving
version: 1.2.10
homepage: https://clawic.com/skills/self-improving
description: Self-reflection + Self-criticism + Self-learning + Self-organizing memory. Agent evaluates its own work, catches mistakes, and improves permanently. Use before starting work and after responding to the user.
changelog: "Sharper setup now lists relevant memory before non-trivial work, with a title that highlights proactive self-reflection."
metadata: {"clawdbot":{"emoji":"🧠","requires":{"bins":[]},"os":["linux","darwin","win32"],"configPaths":["~/self-improving/"]}}
---

## When to Use

User corrects you or points out mistakes. You complete significant work and want to evaluate the outcome. You notice something in your own output that could be better. Knowledge should compound over time without manual maintenance.

## Architecture

Memory lives in `~/self-improving/`. If it does not exist, run `setup.md`.

```
~/self-improving/
├── memory.md          # 用户偏好 + 经验教训 (≤100 lines, always loaded)
├── corrections.md     # 纠错日志 (append-only)
└── archive/           # 归档的旧笔记
```

**⚠️ 不重复原则**: 项目状态、硬件配置、命令参考写在 workspace (MEMORY.md, TOOLS.md, skills/*/SKILL.md)，不写在这里。这里只记录用户偏好、交互风格和经验教训。

## Quick Reference

| Topic | File |
|-------|------|
| Setup guide | `setup.md` |
| Memory template | `memory-template.md` |
| Learning mechanics | `learning.md` |
| Security boundaries | `boundaries.md` |
| Scaling rules | `scaling.md` |
| Memory operations | `operations.md` |
| Self-reflection log | `reflections.md` |

## Detection Triggers

Log automatically when you notice these patterns:

**Corrections** → add to `corrections.md`, evaluate for `memory.md`:
- "No, that's not right..."
- "Actually, it should be..."
- "You're wrong about..."
- "I prefer X, not Y"
- "Remember that I always..."
- "I told you before..."
- "Stop doing X"
- "Why do you keep..."

**Preference signals** → add to `memory.md` if explicit:
- "I like when you..."
- "Always do X for me"
- "Never do Y"
- "My style is..."
- "For [project], use..."

**Pattern candidates** → track, promote after 3x:
- Same instruction repeated 3+ times
- Workflow that works well repeatedly
- User praises specific approach

**Ignore** (don't log):
- One-time instructions ("do X now")
- Context-specific ("in this file...")
- Hypotheticals ("what if...")

## Self-Reflection

After completing significant work, pause and evaluate:

1. **Did it meet expectations?** — Compare outcome vs intent
2. **What could be better?** — Identify improvements for next time
3. **Is this a pattern?** — If yes, log to `corrections.md`

**When to self-reflect:**
- After completing a multi-step task
- After receiving feedback (positive or negative)
- After fixing a bug or mistake
- When you notice your output could be better

**Log format:**
```
CONTEXT: [type of task]
REFLECTION: [what I noticed]
LESSON: [what to do differently]
```

**Example:**
```
CONTEXT: Building Flutter UI
REFLECTION: Spacing looked off, had to redo
LESSON: Check visual spacing before showing user
```

Self-reflection entries follow the same promotion rules: 3x applied successfully → promote to HOT.

## Quick Queries

| User says | Action |
|-----------|--------|
| "What do you know about X?" | Search memory.md + corrections.md |
| "What have you learned?" | Show last 10 from `corrections.md` |
| "Show my patterns" | List `memory.md` |
| "Memory stats" | Show file sizes |
| "Forget X" | Remove from memory.md and/or corrections.md (confirm first) |

## Memory Stats

On "memory stats" request, report:

```
📊 Self-Improving Memory

memory.md: X entries (偏好 + 经验教训)
corrections.md: X entries
archive/: X files

Recent activity (7 days):
  Corrections logged: X
```

## Core Rules

### 1. Learn from Corrections and Self-Reflection
- Log when user explicitly corrects you
- Log when you identify improvements in your own work
- Never infer from silence alone
- After 3 identical lessons → ask to confirm as rule

### 2. Single Responsibility
| 内容 | 写在哪里 |
|------|----------|
| 用户偏好、交互风格、经验教训 | `~/self-improving/memory.md` |
| 纠错记录 | `~/self-improving/corrections.md` |
| 项目状态、架构、硬件配置 | workspace `MEMORY.md`, `TOOLS.md` |
| 命令调用方式 | workspace `skills/*/SKILL.md` |
| 操作日志 | workspace `memory/logs/YYYY-MM-DD.md` |

### 3. Automatic Promotion/Demotion
- Pattern used 3x in 7 days → promote to memory.md
- Pattern unused 30 days → consider removing
- Never delete without asking

### 4. Namespace Isolation
- self-improving: 偏好、经验教训、纠错
- workspace: 项目状态、命令参考、操作日志
- 禁止在 self-improving 中记录 workspace 已有的信息

### 5. Conflict Resolution
When patterns contradict:
1. Most specific wins (project > domain > global)
2. Most recent wins (same level)
3. If ambiguous → ask user

### 6. Compaction
When file exceeds limit:
1. Merge similar corrections into single rule
2. Archive unused patterns
3. Summarize verbose entries
4. Never lose confirmed preferences

### 7. Transparency
- Every action from memory → cite source: "Using X (from projects/foo.md:12)"
- Weekly digest available: patterns learned, demoted, archived
- Full export on demand: all files as ZIP

### 8. Security Boundaries
See `boundaries.md` — never store credentials, health data, third-party info.

### 9. Graceful Degradation
If context limit hit:
1. Load only memory.md (HOT)
2. Load relevant namespace on demand
3. Never fail silently — tell user what's not loaded

## Scope

This skill ONLY:
- Learns from user corrections and self-reflection
- Stores preferences in local files (`~/self-improving/`)
- Reads its own memory files on activation

This skill NEVER:
- Accesses calendar, email, or contacts
- Makes network requests
- Reads files outside `~/self-improving/`
- Infers preferences from silence or observation
- Modifies its own SKILL.md

## Related Skills
Install with `clawhub install <slug>` if user confirms:

- `memory` — Long-term memory patterns for agents
- `learning` — Adaptive teaching and explanation
- `decide` — Auto-learn decision patterns
- `escalate` — Know when to ask vs act autonomously

## Feedback

- If useful: `clawhub star self-improving`
- Stay updated: `clawhub sync`
