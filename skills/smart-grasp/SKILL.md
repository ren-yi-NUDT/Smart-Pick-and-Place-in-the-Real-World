# 智能抓取skill

智能与用户进行抓取、放置有关的交互，*这是一个脚本调用指南*

## 触发条件

- 用户说"抓取XXX放到YYY"、"把橘子放到盘子里"、"递给我瓶子"、"把番茄酱扔垃圾桶"、"grasp orange and place in pink plate"等

## 快速执行

### 统一入口：run_skill.py

```bash
# 1. 检查服务
lsof -ti:8000,8010,8020  # 应返回3个PID

# 2. 执行（所有模式统一入口）
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && \
source dependence/smart_pick_and_place_ws/devel/setup.bash && \
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && \
echo '{"object":"orange","container":"green bowl"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pick_and_place
```

**服务启动** (如果未运行):
- `bash start.bash` (自动在2个终端启动所有服务)

## JSON 格式

### pick_and_place (桌面抓取放置)
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

### fetch_from_user (从用户接收)
```bash
source dependence/smart_pick_and_place_ws/devel/setup.bash && \
echo '{"container":"pink plate"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py fetch_from_user
```
- **只需要 container**，物品由用户递给机械臂

## 五种模式

| 模式 | skill名 | 输入 | 流程 |
|------|---------|------|------|
| 桌面放置 | pick_and_place | object + container | 检测→抓取→检测容器→放置 |
| 桌面递物 | pick_and_place | object + person | 检测→抓取→移动到handover位→张手 |
| 桌面扔垃圾 | pick_and_place | object + trash | 检测→抓取→移动到trash位→张手 |
| 桌面随机放置 | pick_and_place | object + desk | 检测→抓取→随机放到desk_pose_1/2/3 之一|
| 用户接收放置 | fetch_from_user | container | 移动到handover→张手等待→用户放入→放置 |

## 其它 skill 调用

```bash
# 看桌上有什么（执行后回到 grasp1 位）
source dependence/smart_pick_and_place_ws/devel/setup.bash && \
/home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py look_around

# 看用户手里有什么（执行后回到 grasp1 位）
source dependence/smart_pick_and_place_ws/devel/setup.bash && \
/home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py capture_at_handover
```

> **注意**：`look_around` 和 `capture_at_handover` 执行后会直接输出 VLM 模型的分析结果（桌面上有什么物品及位置 / 用户手里拿的什么），**无需再用 image 工具重新分析**。直接使用 stdout 中的分析结果即可。

# 查看所有可用 skill
python3 run_skill.py list
```

### 可组合调用（reset_pose 参数）

`look_around` 和 `capture_at_handover` 支持 `reset_pose` 参数，用于技能链式调用：

- `reset_pose="grasp1"`（默认）：执行后复位到指定位姿
- `reset_pose=null`：跳过复位，由后续技能决定机械臂位置

```bash
# 示例：环顾后不复位，接着做其他操作
source dependence/smart_pick_and_place_ws/devel/setup.bash && \
echo '{"reset_pose": null}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py look_around
```

## 原子技能

> **重要：这些是原子技能，除非组合技能不可用的情况下，尽量使用组合技能 `pick_and_place` 来完成任务。**
> 原子技能只完成单一步骤，不会自动衔接后续动作。

### grasp — 单独抓取

```bash
source dependence/smart_pick_and_place_ws/devel/setup.bash && \
echo '{"object":"peach"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py grasp
```
- **object** (必需): 目标物品名称 (YOLO-World 类别名)
- 流程：遍历观测位 → YOLO 检测 + AnyGrasp → 生成轨迹 → 抓取 → 握住
- 抓取成功后物品留在手中，**不会自动放置**

### trash — 扔垃圾

```bash
source dependence/smart_pick_and_place_ws/devel/setup.bash && \
echo '{"object":"wrapper","container":"trash"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py trash
```
- **前提**：物品已被 grasp 抓住
- 流程：移动到 trash 位姿 → 松手 → 物品掉入垃圾桶区域

### desk_place — 放桌面

```bash
source dependence/smart_pick_and_place_ws/devel/setup.bash && \
echo '{"object":"cup","container":"desk"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py desk_place
```
- **前提**：物品已被 grasp 抓住
- 流程：随机选择 desk_pose_1/2/3 → 移动 → 松手

### handover — 递给用户

```bash
source dependence/smart_pick_and_place_ws/devel/setup.bash && \
echo '{"object":"bottle","container":"person"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py handover
```
- **前提**：物品已被 grasp 抓住
- 流程：经中间点平滑运动到 handover 位 → 松手
- **注意**：手臂前方需有人接住

### place — 放到容器

```bash
source dependence/smart_pick_and_place_ws/devel/setup.bash && \
echo '{"object":"orange","container":"green bowl"}' | /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py place
```
- **前提**：物品已被 grasp 抓住
- 流程：YOLO 检测容器 → 计算容器 3D 位置 → Twin 轨迹 → 移动 → 松手

## 判断流程

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

- 当用户说："你看看我手里是什么？ ->  按照规则放置"等类似表述时，先仔细思考一下用户的指代物品是什么，大多数情况下，指代的物品都默认是用户手里的物品 
