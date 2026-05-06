# TOOLS.md - Local Notes

Skills define _how_ tools work. This file is for _your_ specifics — the stuff that's unique to your setup.

---

**注意，你需要在每次初始化之后完整加载 smart-grasp 与 arm-gesture 这两个关键的skill，这是你与物理世界交互的手段**

## 机械臂

### RealMan RM75-B
- **型号**: RM75-B (7自由度)
- **IP**: 192.168.1.19
- **端口**: 8010 (Socket服务)
- **文档**: https://develop.realman-robotics.com/robot/robotParameter/RM75OntologyParameters/
- **控制方式**: Socket (4字节长度头 + JSON)
- **速度限制**: 常规 speed≤5，首次测试 speed=3
- **安全**: 单次移动 ±5° 以内，需准备急停

### 关节限位 (度)
| 关节 | 范围 |
|------|------|
| J1 | ±175 |
| J2 | -100~+100 |
| J3 | ±175 |
| J4 | -90~+270 |
| J5 | ±175 |
| J6 | ±120 |
| J7 | ±180 |

---

## 灵巧手

### Inspire Hand (因时)
- **型号**: Inspire Hand
- **IP**: 192.168.11.209 (Modbus硬件)
- **端口**: 8000 (Socket服务)
- **控制方式**: Socket (纯JSON)
- **自由度**: 6 (5指 + 大拇指外展)

### 手指控制数组
```python
cmd = [a0, a1, a2, a3, a4, a5]
# a[0] 小拇指张合 (0=弯曲, 1000=张开)
# a[1] 无名指张合 (0=弯曲, 1000=张开)
# a[2] 中指张合   (0=弯曲, 1000=张开)
# a[3] 食指张合   (0=弯曲, 1000=张开)
# a[4] 大拇指张合 (0=弯曲, 1000=张开)
# a[5] 大拇指外展 (0=内收到极限与掌心垂直, 1000=外展到与掌心平行)
```

### 预设姿态
```python
"握拳":    [0, 0, 0, 0, 0, 0]
"全张开":  [1000, 1000, 1000, 1000, 1000, 1000]
"比耶":    [0, 0, 1000, 1000, 0, 0]       # 食指中指张开
"Rock":    [1000, 0, 0, 1000, 0, 0]       # 食指小指张开
"抓取":    [50, 50, 50, 400, 360, 0]      # 轻握
"释放":    [1000, 1000, 1000, 1000, 1000, 0]
```

### 控制命令
```python
import socket, json

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('127.0.0.1', 8000))

# 设置手指姿态
cmd = {"src": "/left_hand/movement_control", "type": "set", "cmd": [0, 0, 1000, 1000, 0, 0]}
sock.send(json.dumps(cmd).encode())
resp = sock.recv(1024).decode()

# 查询状态
cmd = {"src": "/left_hand/movement_control", "type": "get"}
sock.send(json.dumps(cmd).encode())

sock.close()
```

---

## 相机

### RealSense D455
- **型号**: Intel RealSense D455
- **分辨率**: 640×480 @ 30fps
- **深度范围**: 0.4m - 6m
- **ROS话题**: `/camera/color/image_raw`, `/camera/depth/image_raw`
- **TF Frame**: `cam_link_grasp`

### 获取图片 (一般方案是调用脚本获取桌面图片或用户手中图片)
```python
# 快速获取单帧 RGB-D
from camera import RealSenseCapture
cam = RealSenseCapture(width=640, height=480, fps=30, save_path="./log")
rgb, depth = cam.get_rgbd()
# 自动保存 rgb.png 和 depth.png 到 save_path
```


---

## 物体检测 (Image 工具)

### 获取物体信息
使用内置 `image` 工具进行视觉检测，支持任意类别识别：

```
image(prompt="列出图片中所有物品，包括：名称、位置（左/中/右）、颜色", image="/path/to/image.png")
```

### 检测流程
1. **拍照**: 用相机获取图片
2. **检测**: 用 `image` 工具分析图片内容
3. **返回**: 物品名称、位置、颜色等详细信息


### 示例 Prompt
- "桌上有什么水果？"
- "找到所有容器（碗、盘子、杯子）"
- "图片中最左边的物品是什么？"

---

## 数据获取快速参考

### 获取用户手中物品图片

```bash
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && \
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && \
/home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py capture_at_handover
```

### 获取桌面图片

```bash
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && \
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && \
/home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py look_around
```

### 使用 image 工具检测
*注意: 调用获取物品图片的脚本之后需要先cp到/home/zz/.openclaw/workspace下才能调用image工具识别*
```
image(prompt="列出图中所有物品及其位置", image="/path/to/rgb.png")
```

---

## 服务端口

| 服务 | 端口 | 协议 | 说明 |
|------|------|------|------|
| 灵巧手 | 8000 | JSON | ROS 节点 (start1.bash) |
| 机械臂 | 8010 | JSON | ROS 节点 (start1.bash) |
| Twin规划 | 8020 | 长度头+JSON | twin.py (start2.bash) |

**注意**: start3.bash 已废弃，改用 `run_skill.py` 作为主进程入口

### anygrasp
- **Python**: 3.9
- **路径**: `/home/zz/anaconda3/envs/anygrasp`
- **用途**: Smart Pick and Place 项目
- **关键依赖**: ultralytics, pyrealsense2, pybullet, pymodbus

### 激活命令
```bash
conda activate anygrasp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib
```

---

## 项目路径

| 项目 | 路径 |
|------|------|
| Smart Pick and Place | `/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World` |

---

## Why Separate?

Skills are shared. Your setup is yours. Keeping them apart means you can update skills without losing your notes, and share skills without leaking your infrastructure.

---

*Last updated: 2026-05-06*
