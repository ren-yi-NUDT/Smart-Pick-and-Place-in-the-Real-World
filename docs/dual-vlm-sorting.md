# 双臂 VLM 视觉分拣

入口脚本：\`tools/dual_vlm_sorting.py\`

当前默认任务是水果/蔬菜分拣：水果送到粉色盘子，蔬菜送到蓝色盘子。对应配置为
\`configs/dual_vlm_sorting_real_fruit_vegetable.json\`；如需其他分类，再通过
\`--task-config\` 指定任务配置。

脚本将流程拆成四层：

1. 两个机械臂按配置的观察位姿采集多视角 RGB-D；默认左臂复用 `look_around` 的 `grasp1`～`grasp4`，并把每个视角的原图、深度图、VLM JSON 和带框图保存到本次运行目录。
2. VLM 只输出物体、来源平台/盒子和目标容器的语义清单，不输出关节角或运动指令。
3. 初始 VLM 清单只生成“哪个机械臂抓哪个物体、放入哪个容器”的任务队列；抓取位姿不在全场景规划阶段缓存。
4. 执行每个任务时调用对应的单臂视觉抓取：该臂自己的相机重新采集 RGB-D，AnyGrasp 生成位姿，Twin 校验，夹爪确认抓取；成功回 home 后才允许另一臂继续。

### 多视角观测

`observation_poses` 每侧既可以写一个位姿名（兼容旧配置），也可以写位姿列表：

```json
{
  "observation_poses": {
    "left": ["grasp1", "grasp2", "grasp3", "grasp4"],
    "right": [
      {"pose": "observe_left_arm", "tag": "center"},
      {"pose": "observe_left_arm", "tag": "j1_plus8", "joint_offsets_deg": {"J1": 8.0}},
      {"pose": "observe_left_arm", "tag": "j1_minus8", "joint_offsets_deg": {"J1": -8.0}}
    ]
  }
}
```

观测列表也支持以已录制位姿为基准的有限关节偏移。右臂默认在
`observe_left_arm` 附近做 J1 ±8° 小范围扫描，降低物体贴近图像边界导致的漏检；
偏移受 `safety.max_observation_offset_deg` 限制，现场更换相机或改变安装后仍应重新验证手眼标定。
真机水果/蔬菜配置默认融合左右相机作为全局场景清单来源；这些视角只用于初始清单、去重和容器定位，
不会直接把另一只臂的相机数据用于当前机械臂的抓取位姿。可通过 `scene_inventory_sides` 调整清单来源。

机械臂只在切换观察位姿时移动；到达每个位姿后采集并分析一张 RGB-D。每次运行的 `capture_manifest.json` 会记录视角、时间戳、图像、深度、VLM 清单和标注图路径；`scene_inventory.json` 会把 `object_1` 这类内部编号映射到真实物品名称、分类和来源视角。初始 `plan.json` 的 `arm_tasks` 明确记录“左臂/右臂抓取什么、放入哪个容器”，抓取和放置轨迹在对应任务执行时再生成。不同观察位姿下的同一物体以校准后的三维位置为主、语义标签为辅进行保守合并；合并后的分类采用多视角多数结果。

执行阶段采用 home 同步流水线：一只臂调用自己的单臂视觉抓取并回到 home 后，另一只臂才开始调用自己的单臂视觉抓取；第二只臂回到 home 后，第一只臂才使用刚才成功的抓取姿态规划并执行放置。每次闭合夹爪都会检查是否夹到物体，并在回 home 后再次确认持物；失败时单臂视觉抓取会恢复、重新采集并尝试其他抓取候选。放置轨迹从 home 位姿开始由 Twin 校验，放置后打开夹爪并确认，再回 home。任一任务失败都会停止后续任务，并尝试打开左右夹爪；全部动作成功后，双臂再次回到 home 并确认左右夹爪完全打开。

## 仿真

先启动双臂仿真（现在会同时启动左右 Twin：8032/8033）：

\`\`\`bash
bash start_sim.bash
\`\`\`

仿真场景保持两臂基座不动；平台布置在左右侧工作区，平台边长为 0.20 m，源平台与目标平台之间留有间隔，并针对固定观测相机调整了位置。真机部署时应按实际平台位置重新标定，不要直接套用这些仿真坐标。

只拍照、识别和规划，不执行抓取：

\`\`\`bash
SIM_MODE=1 /home/zz/anaconda3/envs/anygrasp/bin/python \\
  -m tools.dual_vlm_sorting --sim --plan-only --yes
\`\`\`

确认仿真计划和日志后再执行：

\`\`\`bash
SIM_MODE=1 /home/zz/anaconda3/envs/anygrasp/bin/python \\
  -m tools.dual_vlm_sorting --sim --execute --yes
\`\`\`

## 真机

真机第一次只运行 --plan-only。脚本会先检查两臂 SDK 健康状态；任一机械臂存在错误状态时会拒绝继续：

\`\`\`bash
/home/zz/anaconda3/envs/anygrasp/bin/python \\
  -m tools.dual_vlm_sorting --plan-only
\`\`\`

确认 log/dual_vlm_sort_*/plan.json、带框图、目标容器和左右臂任务队列均正确后，现场清空工作区，再显式确认执行：

\`\`\`bash
DUAL_SORT_REAL_CONFIRM=1 \\
/home/zz/anaconda3/envs/anygrasp/bin/python \\
  -m tools.dual_vlm_sorting --execute --real-confirm
\`\`\`

真机执行中若抓取或放置失败，脚本会停止后续任务，并执行异常收尾尝试打开两侧夹爪；原始失败原因会保留在终端输出中。

## 修改识别规则

编辑 configs/dual_vlm_sorting_real_fruit_vegetable.json：

- groups：定义任意分类，例如 cool/warm、fruit/vegetable 或具体品类；
- destination_role：把每类物体映射到目标容器角色；
- destinations：定义目标平台、盒子、篮子等容器的语义标签；
- arm_lanes：定义左右臂允许并行的基座坐标工作区；默认支持旧的 `y_min/y_max`，也支持通过 `axis: "x"` 配置 `x_min/x_max`；
- safety.allow_parallel：设为 false 可强制所有动作串行。

VLM 适配入口是 VLMScenePlanner.detect()。它接收一张 RGB 图并应返回如下语义 JSON；坐标可用 0~1000 框：

\`\`\`json
{
  "objects": [
    {
      "id": "obj_1",
      "label": "red apple",
      "group": "warm",
      "source_surface_id": "src_1",
      "confidence": 0.92,
      "box": [100, 200, 260, 380]
    }
  ],
  "destinations": [
    {
      "id": "dst_1",
      "role": "warm_destination",
      "label": "yellow box",
      "confidence": 0.95,
      "box": [700, 200, 950, 450]
    }
  ],
  "source_surfaces": [
    {
      "id": "src_1",
      "label": "green platform",
      "confidence": 0.9,
      "box": [50, 100, 500, 480]
    }
  ]
}
\`\`\`

VLM 不参与运动学和安全决策。真实部署仍需使用正确的相机内参、手眼标定、Twin 模型、工作区 lane 和现场碰撞验证；仅靠图像框不能对任意机械臂姿态作数学意义上的绝对无碰撞保证。
