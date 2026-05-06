# MEMORY.md — Long-Term Memory

## About 用户
- **领域**: 机器人操控、具身智能
- **项目**: Smart Pick and Place 智能抓取放置系统
- **Timezone**: Asia/Shanghai

---

## Active Projects

### Smart Pick and Place
**路径**: `/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World`

**功能**: JSON 控制的智能抓取放置

**架构**:
```
终端1: bash start1.bash  # ROS服务 (8000灵巧手, 8010机械臂)
终端2: bash start2.bash  # Twin IK服务 (8020)
执行: echo '{"object":"orange","container":"green bowl"}' | python planner.py
```

**模式**:
- 抓取放置: `{"object":"xxx","container":"yyy"}`
- 人机递物: `{"object":"xxx","container":"person"}`
- 扔垃圾: `{"object":"xxx","container":"trash"}`
- 用户接收放置: `{"container":"yyy"}` (planner_fetch_from_user.py)


**决策流程**（不写死规则！）:
1. 拍照 → `capture_at_handover.py --no-return`
2. 检测 → `detect_and_decide.py -i <image> -j`
3. **询问用户**要放到哪里
4. 执行 → `echo '{"container":"xxx"}' | fetch_from_user.py`

**状态**: ✅ 生产可用 (2026-03-24)

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
- **新建文件位置**: 在代码库下新建 `/openclaw` 目录

---


*Last updated: 2026-03-24*
