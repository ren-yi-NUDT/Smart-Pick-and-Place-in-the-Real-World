# TOOLS.md - Local Notes（历史模板，源信息以项目根目录 README/COMMANDS 为准）

Skills define _how_ tools work. This file is for _your_ specifics — the stuff that's unique to your setup.

---

## 机械臂 ×2（双臂系统）

### 主手 = 左臂 (精细操作 / 抓取 / 递物)

- **型号**: RealMan RM75-B (7 自由度)
- **IP**: 192.168.1.19
- **端口**: 8010 (Socket 服务)
- **文档**: https://develop.realman-robotics.com/robot/robotParameter/RM75OntologyParameters/
- **控制方式**: Socket (4 字节大端长度头 + JSON)
- **末端**: Robotiq 夹爪（:8002）
- **速度限制**: 常规 `speed ≤ 5`，自学 `speed ≤ 10`

### 副手 = 右臂 (支撑 / 固定 / 接收)

- **型号**: RealMan RM75-B (7 自由度)
- **IP**: 192.168.1.18
- **端口**: 8011 (Socket 服务)
- **控制方式**: Socket (4 字节大端长度头 + JSON) — 协议同主手
- **末端**: 夹爪（modbus 控制，集成在 arm SDK 内）
- **速度限制**: 同主手

> **双臂协调原则**：主手先动 → 副手跟进 → 主手释放。不允许同时异动。详见 `skills/dual-arm-teaching/SKILL.md`。

---

## Robotiq 夹爪

### Robotiq 85

- **型号**: Robotiq 85
- **连接**: 本机串口（Modbus）
- **端口**: 左臂 :8002，右臂 :8001
- **控制方式**: Socket（4 字节大端长度头 + JSON）

---

## 相机

### RealSense D455

- **型号**: Intel RealSense D455
- **分辨率**: 640×480 @ 30fps
- **深度范围**: 0.4m - 6m
- **ROS 话题**: `/camera/color/image_raw`, `/camera/depth/image_raw`
- **TF Frame**: `cam_link_grasp`

---

## 推理后端

| 服务 | 端口 | 用途 |
|---|---|---|
| 孪生推理 (PyBullet) | 8020 | IK 求解 + 轨迹生成 + 碰撞检测 |
| AnyGrasp 抓取检测 | 8030 | top-50 抓取候选生成 |

---

## 端口速查表

| 端口 | 服务 | 协议 |
|---|---|---|
| 8001/8002 | Robotiq 夹爪 | 4 字节头 + JSON |
| 8010/8011 | 机械臂桥接 | 4 字节头 + JSON |
| 8020/8021 | Twin IK | 4 字节头 + JSON |
| 8030 | AnyGrasp | 二进制（4 字节头 + JSON + depth + rgb） |

**启动检查**：`lsof -ti:8001,8002,8010,8011,8020,8021,8030` 应返回已启动服务的 PID。

---

## 项目路径

| 项目 | 路径 |
|------|------|
| Smart Pick and Place | `/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World` |
| Conda 环境 | `/home/zz/anaconda3/envs/anygrasp/bin/python` |
| cuDNN 库 | `/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib` |

---

## Why Separate?

Skills are shared. Your setup is yours. Keeping them apart means you can update skills without losing your notes, and share skills without leaking your infrastructure.

---

*Last updated: 2026-07-08（双臂版）*
