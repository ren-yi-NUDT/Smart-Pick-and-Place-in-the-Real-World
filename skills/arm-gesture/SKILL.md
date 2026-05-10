# arm-gesture Skill

机械臂位姿执行与动作序列播放。结合预录制好的位姿，可以实现表演性动作。

## 触发条件

- 用户说"做个动作"、"敬礼"、"挥手"、"跳舞"等表演性动作
- 用户说"执行动作序列"
- 用户说"列出所有位姿"

## 脚本位置

统一入口：`/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World/run_skill.py`

**必须使用 anygrasp conda 环境的 Python**：
```bash
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && \
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && \
/home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pose_execute
```

## 执行动作序列（从 stdin 传入）

```bash
echo '{"sequence": [{"arm": "home", "hand": "open"}, {"arm": "dance_1", "hand": "peace"}]}' | \
  /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pose_execute
```

### Sequence JSON 格式

```json
{
  "sequence": [
    {"arm": "home", "hand": "open", "delay": 0.8},
    {"arm": "seg_Hail_1", "hand": "open", "delay": 0.6},
    {"arm": "seg_Hail_2", "hand": "peace", "delay": 1.5},
    {"arm": "home", "hand": "open", "delay": 0.5}
  ]
}
```

| 字段 | 说明 |
|------|------|
| **arm** | 位姿名称（必须存在于 recorded_poses.json） |
| hand | 手势（预设名/数组，可选） |
| delay | 延时秒数（默认 0.5） |
| speed | 移动速度 0-100（默认 30） |

⚠️ **重要**: 位姿字段必须用 `arm`，不能用 `name`！
- ✅ 正确: `{"arm": "dance_1", "delay": 0.4}`
- ❌ 错误: `{"name": "dance_1", "delay": 0.4}` （会被忽略，机械臂不动）

## 执行单一位姿

```bash
echo '{"command": "play", "name": "home"}' | \
  /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pose_execute

echo '{"command": "play", "name": "home", "speed": 50}' | \
  /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pose_execute
```

## 列出所有位姿

```bash
echo '{"command": "list"}' | \
  /home/zz/anaconda3/envs/anygrasp/bin/python run_skill.py pose_execute
```

## 手势预设

| 预设名 | 说明 | 手指数值 |
|--------|------|----------|
| open | 全张开 | [1000,1000,1000,1000,1000,500] |
| close | 握拳 | [0,0,0,0,0,0] |
| peace | 比耶 | [0,0,1000,1000,0,0] |
| rock | Rock手势 | [1000,0,0,1000,0,0] |
| pointing | 指向 | [0,0,0,1000,0,0] |
| thumbs_up | 竖大拇指 | [0,0,0,0,1000,800] |
| ok | OK手势 | [800,800,800,150,150,400] |
| grab | 抓取姿态 | [50,50,50,100,100,0] |
*注：执行完动作序列需要把手势恢复到全张开*

**手指数组**: `[小指, 无名指, 中指, 食指, 拇指, 拇指外展]` (0=弯曲, 1000=张开)

## 已录制位姿 (2026-03-23)

### 基础
| 名称 | 用途 |
|------|------|
| home | 初始位置 |
| grasp-ready | 抓取准备位置 |
*注：动作序列执行之前要进入home位置，序列结束后先进入home再进入grasp-ready位置*

### 挥手
| 名称 | 用途 |
|------|------|
| wave_to_user_left | 向用户挥手（左） |
| wave_to_user_right | 向用户挥手（右） |
| wave_to_watcher_left | 向旁观者挥手（左） |
| wave_to_watcher_right | 向旁观者挥手（右） |
| wave_another_pose_left | 挥手变体（左） |
| wave_another_pose_right | 挥手变体（右） |

### 跳舞
| 名称 | 用途 |
|------|------|
| dance_1 | 跳舞动作 1 |
| dance_2 | 跳舞动作 2 |
| dance_3 | 跳舞动作 3 |
| dance_4 | 跳舞动作 4 |
| dance_5 | 跳舞动作 5 |

### 指向
| 名称 | 用途 |
|------|------|
| point_to_user | 指向用户 |
| point_to_person_at_user_right | 指向用户右侧的人 |
| point_to_person_at_user_right_more | 指向用户右侧更远处 |

### 敬礼
| 名称 | 用途 |
|------|------|
| seg_Hail_1 | 敬礼准备 |
| seg_Hail_2 | 敬礼 |

### 手掌展示
| 名称 | 用途 |
|------|------|
| palm_toward_user | 手掌朝向用户（可用于挥手） |
| palm_toward_user_right | 手掌朝向用户右侧（可用于挥手） |
| palm_toward_user_right_more | 手掌朝向用户右侧更远 |

## 前置条件

- **ROS 服务已启动**: `bash start.bash`
- **机械臂已连接**: IP 192.168.1.19

## 安全规则

- 机械臂速度建议 ≤30
- 确保机械臂周围无障碍物
- 准备好急停按钮

## 注意事项

- **delay 设置**: 影响观感的连续动作 delay 应较短 (0.3~0.5s)
- **位姿文件**: 默认存储在 `./recorded_poses.json`，不需要主动读取这个位姿文件，除非动作执行失败。
- **手势配合**: 执行动作序列时应搭配合适的手势来完成
- **动作序列结束后**: 此时相机不会指向桌面或用户，只能用 `run_skill.py look_around` or `run_skill.py capture_at_handover` 来获取桌面 or 用户手上的图片

## 录制位姿（开发工具）

录制新位姿使用 tools/pose_record.py（需直连机械臂 SDK）：
```bash
cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && \
/home/zz/anaconda3/envs/anygrasp/bin/python tools/pose_record.py record --name <位姿名>
```
