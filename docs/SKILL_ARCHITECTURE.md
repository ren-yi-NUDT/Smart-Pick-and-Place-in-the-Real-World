# Skill Architecture

当前所有注册 Skill 都通过 `Skill.run()` 进入统一生命周期：

```text
prepare -> validate_inputs -> execute -> normalize result -> cleanup
```

## 入口约定

- 新 Skill 实现 `execute(**kwargs)`。
- `run(**kwargs)` 由 `skills.base.Skill` 统一提供。
- 旧的、仍实现 `run()` 的外部 Skill 会由 `Skill.__init_subclass__` 临时适配。
- `run()` 始终返回 `core.skill_runtime.SkillResult`。
- `SkillResult` 保留布尔兼容性：可以继续写 `if not result`。

## 共享上下文

每个 Skill 初始化一个 `RobotContext`，通过同一个 Skill 实例复用：

- 配置和日志
- 左右机械臂、夹爪、相机、Twin 和感知客户端
- 坐标变换、抓取、放置、交接和抽屉 Pipeline

上下文目前通过兼容代理访问原有 Skill 方法，后续可以逐步把 Pipeline 改为只依赖显式接口。

## Pipeline 边界

| Pipeline | 负责内容 |
| --- | --- |
| `GraspPipeline` | RGB-D、VLM/YOLO、AnyGrasp、坐标变换、Twin 抓取规划、抓取验证 |
| `PlacePipeline` | 容器识别、安全区域、放置规划、释放和放置后验证 |
| `HandoverPipeline` | 用户交接和双臂交接 |
| `DrawerPipeline` | 抽屉轨迹回放、固定位置释放和安全撤离 |
| `ObservationPipeline` | 工作区扫描、抽屉观察、用户交接位观察和图像保存 |

Skill 只负责组合这些能力。例如：

```text
pick_and_place = grasp + handover(optional) + place
fetch_from_user = receive_from_user + place
grasp_to_drawer = grasp + dual_handover + drawer.release + drawer.close
```

用户交接必须区分三种动作：

- `handover` 默认是左臂 named-pose 递交；传入 `side:"right"` 时走右臂直接递交轨迹。
- `right_give_to_user` 是已注册的右臂直接递交 Skill，使用右臂 `handover_pose`，到位后张爪，停留 2 秒回 home；执行路径复用 `pose_execute`。
- `receive_user_trajectory` / `receive_and_hold` 是从用户处接物；`dual_handover` 只做左右臂之间的物体转移。

`recorded_trajectories/**/*.json` 只是轨迹资源，不能单独被 Skill Registry 发现；必须有 `skills/*.py` 中的 `@register_skill` 包装类，并在 `skills/__init__.py` 导入。

固定目标（桌面、垃圾桶、抽屉）使用 `PlacePipeline.place_at_named_pose()`；视觉目标使用 `PlacePipeline.run()`。

## 当前保留的兼容层

`pick_and_place.py` 中仍保留若干历史辅助方法，供旧调用方使用，但主路径已转到共享 Pipeline。确认所有外部调用迁移后，可以删除这些方法和 `Skill.__init_subclass__` 兼容适配器。

`tools/dual_vlm_sorting.py` 仍保留独立的复杂任务编排器，同时通过 `skills/dual_vlm_sorting.py` 注册为标准 Skill，因此既支持原 CLI，也支持统一入口：

```bash
python3 run_skill.py dual_vlm_sorting
```
