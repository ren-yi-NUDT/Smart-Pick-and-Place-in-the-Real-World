# MEMORY.md — Long-Term Memory

## About 用户
- **领域**: 机器人操控、具身智能
- **项目**: Smart Pick and Place 智能抓取放置系统
- **Timezone**: Asia/Shanghai

---

## Active Projects

### Smart Pick and Place
**路径**: `/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World`

**功能**: JSON 控制的智能抓取放置，Skill-DB 架构

**架构**:
```
终端1: bash start1.bash  # ROS服务 (8000灵巧手, 8010机械臂)
终端2: bash start2.bash  # Twin IK服务 (8020)
执行: echo '{"object":"orange","container":"green bowl"}' | python run_skill.py pick_and_place
```

**代码库结构**:
```
├── core/                    # 共享基础设施 (arm, hand, camera, twin_client, perception, vlm, transforms, config)
├── skills/                  # Skill 实现 (base.py + 各 skill 文件)
├── tools/                   # 开发工具 (pose_record, get_current_pose)
├── run_skill.py             # 统一入口
├── twin_inference/          # 数字孪生 (独立进程)
├── anygrasp_sdk/            # 抓取检测 SDK
├── smart_pick_and_place_ws/ # ROS 工作空间
└── robot_config.json        # 机器人配置
```

**模式**:
- 抓取放置: `echo '{"object":"xxx","container":"yyy"}' | python run_skill.py pick_and_place`
- 人机递物: `echo '{"object":"xxx","container":"person"}' | python run_skill.py pick_and_place`
- 扔垃圾: `echo '{"object":"xxx","container":"trash"}' | python run_skill.py pick_and_place`
- 用户接收: `echo '{"container":"yyy"}' | python run_skill.py fetch_from_user`
- 场景扫描: `python run_skill.py look_around`
- 动作执行: `echo '{"sequence":[...]}' | python run_skill.py pose_execute`
- 查看所有skill: `python run_skill.py list`

**技能调用指南**: 详见 `skills/smart-grasp/SKILL.md` 和 `skills/arm-gesture/SKILL.md`

**状态**: ✅ 生产可用 (2026-03-24)，重构为 Skill-DB 架构 (2026-05-06)

---

## Hardware

| 设备 | 型号 | IP/端口 | 备注 |
|------|------|---------|------|
| 机械臂 | RM75-B (7DOF) | 192.168.1.19:8010 | J5限位±178° |
| 灵巧手 | Inspire Hand | 192.168.11.209:8000 | 6自由度 |
| 相机 | RealSense D455 | USB | 640×480@30fps |

---

## Decisions & Lessons

### 架构决策
- **stdin JSON 输入** (2026-03-11): 替代文件输入
- **移除词汇表** (2026-03-13): 物品检测改用 image 工具，支持任意类别名称
- **Handover 模式** (2026-03-17): 人机递物专用位姿
- **Trash 模式** (2026-03-18): 扔垃圾专用位姿
- **Skill-DB 架构** (2026-05-06): core/ + skills/ + run_skill.py 统一入口

### 经验教训
- **不依赖记忆推断桌上物品**: 必须实时相机扫描检测
- **检测失败 ≠ 抓取失败**: 区分问题阶段
- **Python 环境**: 必须用 anygrasp conda 环境的 Python
- **ROS 服务**: 不能用 OpenClaw exec 后台启动，必须终端手动启动
- **不写死决策规则**: 检测后应询问用户意图，水果不一定放盘子，垃圾不一定扔垃圾桶

---

## Preferences

- **不重启 gateway**: 需要用户手动操作
- **不修改网络配置**: 不要动网络设置
- **代码库操作限制**: 除非允许，只在指定代码库范围内操作
- **新建文件位置**: 一般不新建文件，如实在有需要（如尝试）在代码库下新建 `/openclaw` 目录

---

*Last updated: 2026-05-06*
