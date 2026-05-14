# MEMORY.md — Long-Term Memory

## About 用户
- **领域**: 机器人操控、具身智能
- **项目**: Smart Pick and Place 智能抓取放置系统
- **Timezone**: Asia/Shanghai

---

## Active Projects

### Smart Pick and Place
**路径**: `/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World`

**功能**: JSON 控制的智能抓取放置，Skill-DB 架构（带世界记忆）

**架构**:
```
终端1+2: bash start.bash  # 自动在2个终端启动 ROS服务(8000,8010) + Twin IK(8020)
终端3(可选): bash start3.bash  # planner.py (高级任务规划)
执行: echo '{"object":"orange","container":"green bowl"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pick_and_place
```

**代码库结构**:
```
├── run_skill.py             # 统一 CLI 入口
├── robot_config.json        # 机器人配置（关节位姿、坐标系名称）
├── recorded_poses.json      # 录制的位姿库
├── planner.py               # 高级任务规划器（独立进程 8020）
├── start.bash / start1-3.bash  # 启动脚本
├── core/                    # 共享基础设施
│   ├── config.py            # 集中配置管理（含 get_pose 统一查询）
│   ├── arm.py               # 机械臂 Socket 客户端 (:8010)
│   ├── hand.py              # 灵巧手 Socket 客户端 (:8000)
│   ├── camera.py            # RealSense RGB-D 采集
│   ├── twin_client.py       # 数字孪生客户端 (:8020)
│   ├── transforms.py        # ROS TF 坐标变换
│   ├── perception.py        # YOLO-World + AnyGrasp 封装
│   ├── vlm.py               # GLM-4.5V 视觉语言模型客户端
│   ├── world_memory.py      # 世界记忆（任务历史、物体位置）
│   └── world_model_critic.py # 世界模型批评器（自评估）
├── skills/                  # Skill-DB
│   ├── base.py              # Skill 基类 + 注册机制 + 懒加载硬件
│   ├── __init__.py          # 导入所有 skill 触发注册
│   ├── pick_and_place.py    # 高级：抓取+放置（handover/trash/desk 内联）
│   ├── fetch_from_user.py   # 高级：从用户接收（trash/desk 内联）
│   ├── look_around.py       # 高级：场景扫描 + VLM 分析
│   ├── capture_at_handover.py # 高级：handover 拍照
│   ├── pose_execute.py      # 位姿/动作序列执行（支持手势）
│   ├── grasp.py             # 独立：视觉抓取
│   ├── place.py             # 独立：视觉放置
│   ├── handover.py          # 独立：递交给用户
│   ├── trash.py             # 独立：扔垃圾
│   └── desk_place.py        # 独立：放桌面
├── memory/                  # 世界记忆系统
│   └── world/               # 任务历史、物体状态、事件日志
├── tools/                   # 开发工具（pose_record, get_current_pose）
├── dependence/              # 第三方依赖
│   ├── twin_inference/      # 数字孪生推理（独立进程）
│   ├── anygrasp_sdk/        # AnyGrasp 抓取检测 SDK
│   ├── yolo_world/          # YOLO-World 模型
│   └── smart_pick_and_place_ws/ # ROS catkin 工作空间
└── smart_pick_and_place_ws/ # 同 dependence/ 中的 ROS 工作空间（冗余）
```

**模式**:
- 抓取放置: `echo '{"object":"xxx","container":"yyy"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pick_and_place`
- 人机递物: `echo '{"object":"xxx","container":"person"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pick_and_place`
- 扔垃圾: `echo '{"object":"xxx","container":"trash"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pick_and_place`
- 放桌面: `echo '{"object":"xxx","container":"desk"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pick_and_place`
- 用户接收: `echo '{"container":"yyy"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py fetch_from_user`
- 场景扫描: `/home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py look_around`
- 动作执行: `echo '{"sequence":[...]}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pose_execute`
- 查看所有skill: `/home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py list`

**技能调用指南**: 详见 `skills/smart-grasp/SKILL.md` 和 `skills/arm-gesture/SKILL.md`

**状态**: ✅ 生产可用 (2026-03-24)，重构为 Skill-DB 架构 (2026-05-06)，2026-05-14 合并 world_memory + planner，保留原版可回退

### 当前文件结构
```
├── core/                     # 共享基础设施
│   ├── world_memory.py       # [新增] 世界记忆系统
│   └── world_model_critic.py # [新增] 世界模型批评器
├── skills/
│   ├── pick_and_place.py     # 原版（稳定工作）
│   ├── pick_and_place_v2.py  # [新增] 新版（世界记忆+批评器）
│   └── world_memory_setup.py # [新增] 世界记忆初始化
├── planner.py                # [新增] 高级任务规划器
├── utils.py                  # [新增] 工具函数
├── transformation.py         # [新增] 坐标变换工具
├── start1.bash / start2.bash / start3.bash  # [新增] 独立启动脚本
└── memory/world/             # [新增] 世界记忆数据
```

**使用方式**：
- `pick_and_place` 原版 → 日常工作使用
- `pick_and_place_v2` 新版 → 试用世界记忆+自我评估
- `planner` 规划器 → `echo '{"task":"..."}' | python run_skill.py planner`（需要 start3.bash）

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
- **世界记忆系统** (2026-05-14): memory/world/ 记录任务历史、物体位置
- **Planner 规划器** (2026-05-14): planner.py 实现高级任务规划 (start3.bash)

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
