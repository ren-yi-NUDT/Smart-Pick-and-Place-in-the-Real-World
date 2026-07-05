# 抓取位姿 180° 翻转歧义消除(右臂)

**日期**: 2026-07-04
**分支**: Double-arm-on-desk
**作者**: Ren Yi + Claude

## 背景

AnyGrasp 对一只夹爪会返回等价的两个抓取位姿:绕 end_effector 局部 Z 轴(接近方向)相差 180°。这是 AnyGrasp 的内在歧义——夹爪关于 ZOY 平面对称,所以这两个位姿在物理上是等价抓取。

但对机械臂 IK 解算器来说,这两个位姿对应的手腕扭转角 J7(RM-75 的第 7 关节,绕手腕局部 Z 旋转)相差 180°。如果 AnyGrasp 恰好返回了"远离 home 的那个"解,IK 会解出接近 ±180° 的 J7,导致:

1. 关节极限附近运动不平稳
2. 路径规划需要绕远(关节空间多绕一圈)
3. 与上一姿态的衔接动作幅度大,易碰撞

## 目标

在位姿送入 IK 之前做一次后处理,挑选使 J7 接近 home(即 |J7| 较小)的那个等价位姿。

## 范围

**仅右臂**(Robotiq 85 夹爪)。左臂用 `_transform_x_axis`,本次不动。`grasp.py` 独立 skill 当前硬编码为左臂,不涉及。

## 现状

`skills/pick_and_place.py:608-617` 已有 `_transform_right_grasp` 方法,做的事**完全一样**(绕局部 Z 翻 180°),但触发条件是"夹爪 Y 轴与世界 Z 同向时翻"——这是早期尝试,本意约束手腕俯仰,但实测对绕手腕 J7 的扭转没直接帮助。

## 设计

### 数学规则

记 `T = transformed_pose_world`(4×4 齐次矩阵)。注意变量名虽叫 "world",实际是 `R_base_link` 帧下的位姿(因为 `T_base_to_cam` 由 `transforms.get_transform_from_frame_to_frame("R_base_link", "R_cam_link_grasp")` 得到)。

其旋转部分 `R = T[:3, :3]` 的列向量为夹爪三个局部轴在 `R_base_link` 帧下的表示:

- `R[:, 0]` = 夹爪 X 轴(夹爪"前方/侧向")
- `R[:, 1]` = 夹爪 Y 轴
- `R[:, 2]` = 夹爪 Z 轴(接近方向)

**检测条件**:`R[0, 0] < 0` → 夹爪 X 轴投影到 base XY 平面后,落在 -X 半平面。

**翻转操作**:`T ← T @ diag(-1, -1, 1, 1)`(绕夹爪局部 Z 轴翻 180°)。X 和 Y 列同时变号,Z 列不变,接近方向保持。由于夹爪关于 ZOY 平面对称,这是物理等价的抓取位姿。

**性质**:翻转后 `R[0, 0]` 符号反转 → 新规则保证翻转后 `R[0, 0] ≥ 0`。

**边界情况**:`R[0, 0] = 0` 时严格不等号 `< 0` 不成立,不翻转。夹爪处于中间姿态(局部 X 轴垂直于 base X),翻不翻都行,留给 IK 解算器自由选择。

### 阈值选择

严格 `< 0`,不引入余量阈值。理由:

- AnyGrasp 输出是稳定的(基于点云的优化解),不会在边界震荡
- 严格判定保证规则是确定性的、可重现的
- 简单

### 替换 vs. 共存

**替换**现有 `_transform_right_grasp`。理由:两个规则都翻 180° 关于局部 Z,约束的是同一个自由度(绕 Z_local 的旋转角)。叠加施加会互相覆盖——一个想翻、另一个不想翻,最终结果由后者决定,等价于只保留后者。所以直接替换,语义更清晰。

### 代码改动

**单文件单方法**:`skills/pick_and_place.py:608-617` 的 `_transform_right_grasp`。

旧实现:

```python
def _transform_right_grasp(self, transformed_pose_world):
    r, p, y = R.from_matrix(transformed_pose_world[:3, :3]).as_euler('xyz', degrees=False)
    y_axis_rotated = rpy_to_vector(r, p, y, axis=[0, 1, 0])
    z_axis_world = np.array([0, 0, 1])
    cos_theta = np.dot(y_axis_rotated, z_axis_world) / (
        np.linalg.norm(y_axis_rotated) * np.linalg.norm(z_axis_world)
    )
    if cos_theta > 0:
        transformed_pose_world = transformed_pose_world @ np.array([
            [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return transformed_pose_world
```

新实现:

```python
def _transform_right_grasp(self, transformed_pose_world):
    """Disambiguate AnyGrasp's 180°-about-Z symmetry for the right arm.

    Pick the orientation where the gripper's local X-axis (projected onto
    R_base_link's XY plane) points into the +X half-space, so IK tends to
    give a small J7 (wrist roll) rather than one rotated ~180° from home.
    """
    if transformed_pose_world[0, 0] < 0:
        transformed_pose_world = transformed_pose_world @ np.array([
            [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return transformed_pose_world
```

**调用点不变**:`_transform_anygrasp_pose`(line 583)在 `side == "right"` 分支继续调用 `_transform_right_grasp`。

**辅助函数**:`rpy_to_vector`(`core/transforms.py`)旧 `_transform_right_grasp` 是它的一个调用点。替换后 `pick_and_place.py` 不再用 `rpy_to_vector`,但 `grasp.py:transform_x_axis`(左臂)仍在用,所以保留。

## 验证

### 单元验证(无需硬件)

构造测试矩阵:

1. **触发翻转**:`R[0, 0] < 0` → 验证翻转后 `R[0, 0] > 0`
2. **不翻转**:`R[0, 0] > 0` → 验证不翻转
3. **边界**:`R[0, 0] = 0` → 验证不翻转(严格 `<` 不触发)
4. **接近方向保持**:翻转前后 `R[:, 2]` 不变
5. **平移分量保持**:翻转前后 `T[:3, 3]` 不变(右乘只影响旋转部分)

### 集成验证(需硬件)

跑一次右臂抓取流水线,观察:

- log 中打印的 IK 解 J7 值
- 改前 vs 改后:|J7| 应明显更小(理想情况下 < 90°)

## 不做的事

- 不动左臂的 `_transform_x_axis`
- 不改 `grasp.py`(独立 skill,左臂专用)
- 不改 URDF(`R_gripper_endeffector_joint` 的 +Z 平移 0.012m 是 TCP 对齐问题,与本次手腕扭转无关)
- 不引入 IK 双解对比方案(几何启发式已足够,且无需新增 IK 调用)
- 不引入阈值余量(严格 `< 0`)
