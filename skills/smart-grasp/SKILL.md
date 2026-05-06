# 智能抓取skill

智能与用户进行抓取、放置有关的交互，*这是一个脚本调用指南*

## 触发条件

- 用户说"抓取XXX放到YYY"、"把橘子放到盘子里"、"递给我瓶子"、"把番茄酱扔垃圾桶"、"grasp orange and place in pink plate"等

## 快速执行

### 模式1: 桌面抓取放置 (planner.py)
```bash
# 1. 检查服务
lsof -ti:8000,8010,8020  # 应返回3个PID

# 2. 执行抓取
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && \
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && \
echo '{"object":"orange","container":"green bowl"}' | /home/zz/anaconda3/envs/anygrasp/bin/python planner.py
```

### 模式2: 从用户手中接收并放置 (planner_fetch_from_user.py)
```bash
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && \
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && \
echo '{"container":"pink plate"}' | /home/zz/anaconda3/envs/anygrasp/bin/python planner_fetch_from_user.py
```

**服务启动** (如果未运行):
- 终端1: `bash start1.bash` (8000灵巧手, 8010机械臂)
- 终端2: `bash start2.bash` (8020 Twin IK)

## JSON 格式

### planner.py (桌面抓取)
```json
{"object": "orange", "container": "green bowl"}  // 放置到容器
{"object": "bottle", "container": "person"}      // 人机递物
{"object": "ketchup", "container": "trash"}      // 扔垃圾
{"object": "apple", "container": "desk"}         // 放到桌面（随机位置）
```
- **object**: YOLO-World 类别名，空格格式 (`pink plate`)
- **container**: 
  - 容器名 (`green bowl`, `pink plate` 等)
  - `person` (人机递物)
  - `trash` / `垃圾桶` (扔垃圾)
  - `desk` / `桌子` / `table` (放到桌面随机位置)

### planner_fetch_from_user.py (从用户接收)
```json
{"container": "pink plate"}  // 放到指定容器
{"container": "trash"}       // 扔垃圾桶
{"container": "desk"}        // 放到桌面
```
- **只需要 container**，物品由用户递给机械臂

## 五种模式

| 模式 | 脚本 | 输入 | 流程 |
|------|------|------|------|
| 桌面放置 | planner.py | object + container | 检测→抓取→检测容器→放置 |
| 桌面递物 | planner.py | object + person | 检测→抓取→移动到handover位→张手 |
| 桌面扔垃圾 | planner.py | object + trash | 检测→抓取→移动到trash位→张手 |
| 桌面随机放置 | planner.py | object + desk | 检测→抓取→随机放到desk_pose_1/2/3 之一|
| 用户接收放置 | planner_fetch_from_user.py | container | 移动到handover→张手等待→用户放入→放置 |


### 抓取请求判断流程
1. **用户要求抓取物品** → 先回想记忆中桌上有什么
2. **记忆中有该物品** → 直接执行抓取
3. **记忆中没有该物品** → 说"我记得桌上没有xxx，我再看一眼" → 拍照确认
4. **确认没有** → 如实告诉用户"没找到xxx"

### 从用户手中接物品判断流程
1. 用户说"接着" / "拿一下" / "帮我拿" → 移动到 handover 位，张手等待
2. 用户放入物品 → 握住
3. **判断用户是否已说明目的地**：
   - 已说明（如"把这个放盘子里"）→ 直接执行
   - 未说明 → 拍照检测物品 → **询问用户** "要把这个xxx放到哪里？"
4. 用户回答 → 执行放置

### 辅助脚本

**fetch_from_user.py** - 接物品并放置（用户已指定容器）
```bash
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && \
echo '{"container":"desk"}' | /home/zz/anaconda3/envs/anygrasp/bin/python fetch_from_user.py
```

### 看用户手里有什么（可拍照并检测图像） (拍照后程序返回图片分析结果，不需要调用image工具)
```bash

# 拍照后不返回，留在 handover 位置（等待用户放入物品）
python3 capture_at_handover.py --no-return

# 拍照后返回 grasp1
python3 capture_at_handover.py

```

### 看看桌上有什么 （可拍照并检测图像）(拍照后程序返回图片分析结果，不需要调用image工具)
```bash                                                                                                                               
  python3 look_around.py     
```



## 用户意图判断

当用户说以下话语时，自动转换为对应模式：

| 用户说 | 转换为 |
|--------|--------|
| "放桌上" / "放桌子上" / "放桌子" | `container: "desk"` |
| "给我" / "递给我" / "我要" | `container: "person"` |
| "扔掉" / "扔垃圾桶" / "丢掉" | `container: "trash"` |


## 硬件配置

| 设备 | 端口 | 协议 |
|------|------|------|
| 灵巧手 (Inspire) | 8000 | 纯JSON |
| 机械臂 (RM75-B) | 8010 | 4字节头+JSON |
| Twin IK | 8020 | 4字节头+JSON |

## 常见问题

| 问题 | 解决 |
|------|------|
| `ModuleNotFoundError: pyrealsense2` | 用 anygrasp conda 环境的 Python |
| `No grasp points found` | 跟用户说抓取失败，请求用户帮助 |
| `Pose not reachable` | 系统自动重试，无需干预 |

## 其它

- 当用户说：“你看看我手里是什么？ ->  按照规则放置”等类似表述时，先仔细思考一下用户的指代物品是什么，大多数情况下，指代的物品都默认是用户手里的物品 
