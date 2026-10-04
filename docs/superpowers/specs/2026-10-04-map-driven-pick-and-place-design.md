# 地图驱动的抓放架构 (Map-Driven Pick and Place)

**日期**: 2026-10-04
**状态**: 待评审

## 背景

现在 `skills/pick_and_place.py` 的抓取阶段和放置阶段各有一份**相同的**四位姿扫描循环
（`pick_and_place.py:53` 和 `:148`），都从 `grasp1` 开始逐个走 `grasp1→grasp4`：

```python
for key, value in self.config.default_traj_js.items():
    if "grasp" not in key:
        continue
    self.control_arm(pose_type=key, speed=30)
    self.rgb, self.depth = self.get_camera_obs()
    ...
```

后果（2026-10-04 实测，放桃子进蓝盘子那一轮）：

- 抓完桃子时臂在 `grasp4`，**刚看见蓝盘子**；放置阶段却从 `grasp1` 重新开始，
  4 次移臂 + 4 次拍照 + 4 次 YOLO 去找几秒前就在看的东西。
- 前两个位姿是**必然白跑**的：日志里两条 `要 blue plate, 但只看到 {'pink'}; 颜色对不上, 不猜`
  就是白跑的证据。
- `core/scene.py` 的场景地图本来就是为这个写的（`look_around` 扫一遍记下每样东西的位置，
  与臂的朝向无关），但它只在四位姿**全失败之后**才查（`pick_and_place.py:167`），
  而且平时是空的 —— 只有单独跑 `look_around` 才会写。

另有一个独立的运动问题：**腕部支路不统一**。`grasp1-4` 录制值 J4 ≈ -164…-172.5°，
离真机 ±177.96° 限位只剩 ~9°，抓取候选需要腕部转动时会被 IK 逼到 ZYZ 翻转分支，
而翻转分支要穿过 ±180° 的限位死区（J4 行程 355.92°，比整圈少 4°），孪生的 `BRANCH_JUMP`
保护因此正确地把候选判成不可达（见 `docs` 之外的分析与 `project-rm65-wrist-branch-j4-limit`）。

2026-10-04 的临时修法是 `Config.reload()` 只把 `grasp*` 翻到 ZYZ 等价解。这**引入了新的
多余运动**：`home` 本身在 J4=-170.5（原支路），从 home 到翻转后的观察位姿要在腕上转
**179°**。实测对照：

| 路径 | 原支路 | 只翻 grasp*（当前） | 全表统一翻 |
|---|---|---|---|
| home → grasp1 | 38.9° | **178.9°** | 38.9° |
| home → place1 | 88.3° | 88.3° | 88.3° |
| home → handover_pose | 103.5° | 103.5° | 103.5° |

## 目标

1. **扫一次建图 → 直奔对的位姿 → 抓 → 放**。每个阶段只访问一个观察位姿。
2. **消除腕部支路跳跃**，让"翻腕 / 360° 旋转"从根上消失。
3. **抓取只试一个候选**，失败即报失败退出。

## 非目标

- **不动真机做抓取验证**（本次只搭架构）。
- **不删检测**。长期路线是先让"抓准放准"稳住，再考虑去掉运行时检测；本设计是那条路线的
  第一段（用地图**选位姿**，不用地图**算位姿**）。
- **不做末尾回访确认**。已讨论并砍掉：由人看桌面即可，不值得引入一套成败判定逻辑。
- 不改 `robot_config.json` 里的录制数值（翻转只在内存里做，见下）。

## 设计

### 1. 关节支路统一

`wrist_headroom_pose` 是 ZYZ 等价变换 `(J4,J5,J6) → (J4+180, -J5, J6+180)`，对合，
且是腕关节空间上的**等距变换**：两个位姿一起翻，J4 与 J6 的差不变、J5 的差只变号，
所以**两两相对距离逐个保持不变**。上表 home→grasp1 的 38.9°→38.9° 就是这个性质。

因此把作用域从 `grasp*` 扩成**整份位姿表**即可：既拿到腕部余量，又不新增任何运动。

改动点（`core/config.py:Config.reload()`）：对 `default_traj_js` 的**每一项**和顶层的
每一个 J1..J6 位姿都过一遍 `wrist_headroom_pose`。磁盘上的 `robot_config.json` 不动 ——
翻转是加载时的内存操作，`tools/teach_record.py` 录下来的仍是原始值。

**必须同步更新的测试**：`tests/test_observation_pose_branch.py::test_config_flips_observation_poses_but_not_others`
现在断言 `handover_pose` **不**被翻。本设计之后它**会**被翻，该断言要反过来
（改成"递送位姿也被翻到同一支路，且翻转前后末端位姿不变"）。

### 2. 地图驱动选位姿

`look_around` 建图时每条记录已经带 `seen_from`（哪些观察位姿看见过它）。用它选位姿：

- **抓取阶段**：`scene_map.find(map, object)` → 取 `seen_from[0]` → 只去那一个位姿，
  在那儿拍照、跑 AnyGrasp、抓。`object` 字段按**整串当名字**查（命令里不写颜色，
  `{"object":"plum"}` 就是一个词），所以 `SCAN_VOCAB` 里的物体词必须是单词，
  这也和实测可用词一致。
- **放置阶段**：`container` 字段拆词，`"blue plate"` → 颜色 `blue` + 名词 `plate`
  （沿用 `_placing_position_from_scene` 现有的 `words[0]` / `words[-1]` 拆法），
  查 `find(map, "plate", "blue")` → 只去那一个位姿，在那儿检测、算放置点。
- **取哪个位姿**：`seen_from[0]`。`scene_map.merge` 按扫描顺序追加，所以 `[0]` 就是
  `grasp1→grasp4` 里**最先**看见它的那个，也就是走路最少的那个。
- **回退**：地图里没有该目标 → 走现在的四位姿循环（**保留不删**，作为保底）。

本阶段只用地图的 `seen_from`。地图里的世界坐标暂时不用 —— 那是路线 B（按坐标算抓取位姿）
的料，先留着。

### 3. 自动建图

`pick_and_place` 开头检查 `./log/scene.json` 的 mtime，超过 `SCENE_MAP_TTL_S`
（新常数，默认 300 秒）就自动扫一遍再继续。

**注意 CLAUDE.md 的硬规矩**：流水线执行中**禁止跨 Skill 实例化**（新实例会另开
arm/hand 的 TCP 连接，破坏阶段衔接）。所以自动扫描**不能**写 `LookAroundSkill().run()`，
必须把扫描逻辑抽成一个**函数**，由 `look_around` 和 `pick_and_place` 共用。

抽法：把 `look_around.py` 的循环 + `_scan_pose` + `_world_point` 提成
`core/scene.py` 里的一个纯函数（入参是 arm / camera / perception / config，不持有硬件），
`look_around.py` 调它，`pick_and_place.py` 也调它。

自动路径**不跑 VLM**。VLM 那段的产出是给人读的场景描述，对建图没有用，而它又慢又花钱。

### 4. 词表对齐

`scene_map.SCAN_VOCAB` 现在存通用词（`peach` / `dragon fruit`），但下指令用的是
YOLO-World 在这张桌上实际认得出的词。2026-10-04 实测：

| 物体 | 可用词 | 置信度 | 不可用 |
|---|---|---|---|
| 桃子 | `plum` | 0.836 | `peach` 只有 0.325；`nectarine` 会误匹配粉盘子 |
| 火龙果 | `pitaya` | 0.166 | `dragon fruit` **零检测**；`red fruit` 从 grasp1 会匹配到橙子 |
| 黄瓜 | `cucumber` | 0.906 | — |
| 橙子 | `orange` | 0.665 | `red fruit` 同框 |
| 柠檬 | `lemon` | 0.761 | — |

`SCAN_VOCAB` 改成这组词。这是**现场标定**的，和白平衡常数 (`CAMERA_WHITE_BALANCE`)
同一性质 —— 换场景、换东西要重探。留一个 `tools/probe_vocab.py` 做重探。

### 5. 抓取只试一个候选

删掉 `pick_and_place.py:80` 的候选循环：

```python
for i in range(len(grasping_pose_world)):
    cprint(f"=================== Checking pose: {i+1} / {len(grasping_pose_world)} ===================", "yellow")
    check = self._execute_grasping_twin_js_2(grasping_pose_world[i], idx=key)
    if check:
        break
```

改成只取分数最高的一个（`_transform_anygrasp_pose` 已按 score 降序返回）。
失败 → `Grasping task failed` → `return False`。

## 验证方案（不动真机）

1. **单元测试**
   - 支路统一：`wrist_headroom_pose` 作用于全表后，任意两位姿的腕部相对距离与翻转前**逐对相等**。
   - 全表 J4 都落在 `abs(J4) <= 90`，且都在真机限位内。
   - `Config()` 加载后 `handover_pose` 等顶层位姿**确实被翻**（改写现有断言）。
   - 地图选位姿：给一份假地图，断言 `pick_and_place` 选出的是 `seen_from[0]`。
   - 回退：空地图时仍走四位姿循环。
   - TTL：地图文件 mtime 新/旧两种情况下，是否触发自动扫描。
2. **纯计算验证**（不需要臂动）
   - 用孪生的正向运动学验：翻转前后每个位姿的 `link_6` 位姿一致。
3. **不做的**：真机抓取、真机移动。
   - `home` 及 place/handover/trash/desk 翻转后的姿态**尚未在真机上跑过**。
     `move_to_named_pose` 走关节空间 movej，**不经过孪生的碰撞检查**，所以这几条
     属于"理论等价、未经实测"。等下次上真机时，按 `grasp1-4` 那样逐个慢速验一遍
     末端位姿再放行。

## 风险

| 风险 | 说明 | 处理 |
|---|---|---|
| 翻转后的 home/place/handover 姿态可能撞桌 | movej 不过孪生碰撞检查 | 下次上真机时慢速逐个验；本设计不动真机 |
| 地图过期 | 建图后东西被移动 | TTL 到期自动重扫；`seen_from` 只用来选位姿，去了之后仍当场检测 |
| 地图词表对不上用户说法 | 用户说"桃子"，地图里是 `plum` | 词表统一到指令词；重探脚本留好 |
| 自动扫描打断流水线 | 违反"禁止跨 Skill 实例化" | 抽成函数共用，不 new Skill |

## 未决

- `SCENE_MAP_TTL_S` 默认 300 秒是拍的，实际上真机上要试多久合适。
