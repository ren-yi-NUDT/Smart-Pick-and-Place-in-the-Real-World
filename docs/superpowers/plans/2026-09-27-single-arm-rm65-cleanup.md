# Single-Arm RM65 Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert only `/home/zz/Desktop/tianyi_single_arm` into a runnable single RM65 system at `192.168.1.18`, without modifying the original project.

**Architecture:** Keep the existing RM65 backend and one left-side skill interface as the sole runtime path. Remove the alternate Igrape backend and dual-arm-only services/configuration, then make the ROS bringup, arm bridge, Twin service, and config consistently use one arm and one camera/gripper. Do not touch `/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World`.

**Tech Stack:** Python, ROS Noetic/catkin, TCP arm/gripper/Twin bridges, JSON configuration, pytest, git worktree.

---

## 执行状态 (2026-09-30)

Task 1–4 的代码改动已全部落地并验证；下方复选框未逐条勾选，以本表为准。

| Task | 状态 | 验证 |
|---|---|---|
| 1 删除 igrape 后端 | 完成 | `core/backends/__init__.py` 只剩 `BACKEND = "rm65"`；`import core.backends` → `rm65 False`；`core/backends/igrape/` 已不存在 |
| 2 统一单臂网络标识 | 完成 | `robot_config.json` = RM65 / 192.168.1.18 / 8080 / 6 DOF；全仓已无 `192.168.1.19`、`8011`、`8021`、`right_arm`（仅剩 `8001` = 保留的 Robotiq 夹爪口） |
| 3 启动与 ROS bringup 单臂化 | 完成 | `bash -n start.bash` 通过；`start.bash` 只起 1 套 bringup/Twin/Azure/夹爪；`bringup.launch` 只含 1 个 `arm_75_bringup.py` + 1 个 `mount_camera.py` |
| 4 去除双臂运行路由 | 完成 | `skills/__init__.py` 只注册 10 个单臂 skill；`twin_inference/robot.py` 只有单个 `Arm`/`ErdaijiRobot`；`python3 -m compileall -q core skills dependence/twin_inference tools` 通过 |
| 5 无硬件验证 | 部分 | 见下 |

**与计划的偏差**（计划里的部分文件名/测试名是起草时的推测，实际未采用）：

- 计划中的 `-k rm65_is_the_only_backend` / `single_arm_network_config` / `single_arm_startup` / `no_dual_runtime` 四个测试，实际合并为 `tests/test_single_arm_static.py` 里的 4 个测试。
- 计划中的 `tools/start_arm_bridge.sh` 从未存在，`start.bash` 直接起各进程，无需该文件。
- `inspire_hand_bringup.py` 节点仍保留在 `bringup.launch` 中（测试断言其存在），未按"旧设备停用"删除。
- Task 5 Step 4（TCP 连通性检查）**未执行**：需机器人上电，未经确认不跑。

**Task 5 结果**：

- `python3 -m pytest -q` → 4 passed（新增 `pytest.ini` 限定 `testpaths = tests`，否则会去收集 `Open-Jev/` 的 69 个外部测试并报错）
- `git diff --check` / `git diff --cached --check` → clean（`rm65.urdf` 第 2 行行尾空格已去除）
- 原仓库 `/home/zz/Code/Smart-Pick-and-Place-in-the-Real-World` 未被本次清理改动；本目录是它的 git worktree（分支 `tianyi_single_arm`），原仓库工作区的改动是它自己的双臂开发内容

---

### Task 1: Remove the unused alternate backend

**Files:**
- Delete: `/home/zz/Desktop/tianyi_single_arm/core/backends/igrape/`
- Modify: `/home/zz/Desktop/tianyi_single_arm/core/backends/__init__.py`
- Test: `/home/zz/Desktop/tianyi_single_arm/tests/`

- [ ] **Step 1: Write the failing import/config test**

Add a focused test asserting that backend selection is fixed to RM65 and that importing the backend factory does not reference Igrape modules:

```python
def test_rm65_is_the_only_backend():
    import core.backends as backends
    assert backends.BACKEND == "rm65"
    assert not hasattr(backends, "IgrapeArm")
```

- [ ] **Step 2: Run the test before cleanup**

Run:

```bash
cd /home/zz/Desktop/tianyi_single_arm
python3 -m pytest tests -q -k rm65_is_the_only_backend
```

Expected: the test identifies the current optional Igrape branch or fails because the cleanup has not been applied.

- [ ] **Step 3: Remove Igrape selection and files**

Delete `core/backends/igrape/`. In `core/backends/__init__.py`, remove the `ROBOT_BACKEND` environment switch and all Igrape imports/branches. Keep only the existing RM65 factory implementations and make `BACKEND = "rm65"`.

- [ ] **Step 4: Run the focused test and import check**

Run:

```bash
cd /home/zz/Desktop/tianyi_single_arm
python3 -m pytest tests -q -k rm65_is_the_only_backend
python3 -c 'import core.backends; print(core.backends.BACKEND)'
```

Expected: the test passes and the command prints `rm65`.

---

### Task 2: Normalize the single-arm network identity

**Files:**
- Modify: `/home/zz/Desktop/tianyi_single_arm/core/config.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/robot_config.json`
- Modify: `/home/zz/Desktop/tianyi_single_arm/core/arm.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/tools/get_current_pose.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/tools/pose_record.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/dependence/smart_pick_and_place_ws/src/rm_65_pkg/src/arm_75_bringup.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/dependence/smart_pick_and_place_ws/src/rm_65_pkg/src/arm_75_rm.py`

- [ ] **Step 1: Add a configuration assertion test**

Add a test that loads the desktop copy's JSON and verifies one arm configuration, IP `192.168.1.18`, bridge port `8010`, and a six-joint RM65 model declaration where the configuration exposes model metadata.

- [ ] **Step 2: Run the test before edits**

Run:

```bash
cd /home/zz/Desktop/tianyi_single_arm
python3 -m pytest tests -q -k single_arm_network_config
```

Expected: FAIL because the checked-out files still contain the old `192.168.1.19` identity and dual-arm keys.

- [ ] **Step 3: Apply the identity changes**

Use `192.168.1.18` everywhere in the single-arm runtime path. Keep the local bridge at `127.0.0.1:8010`; do not change the robot's native SDK port `8080`. Change service naming from `/right_arm/movement_control` to the single-arm service used by the retained left-side skill path, consistently across `core/arm.py`, the bridge response, and skill command builders. Set the arm model/DOF to RM65/six joints in the retained configuration and remove the second arm object rather than leaving a fake duplicate.

- [ ] **Step 4: Validate JSON and focused tests**

Run:

```bash
cd /home/zz/Desktop/tianyi_single_arm
python3 -m json.tool robot_config.json >/dev/null
python3 -m pytest tests -q -k single_arm_network_config
rg -n -i '192\.168\.1\.19|right_arm|8011|8001|8021' core skills tools robot_config.json start.bash dependence/smart_pick_and_place_ws/src/pkg_launch/launch dependence/smart_pick_and_place_ws/src/rm_65_pkg/src
```

Expected: JSON validation passes; the focused test passes; remaining matches are only explicitly documented compatibility text or historical data, not executable single-arm startup/control paths.

---

### Task 3: Make startup and ROS bringup single-arm only

**Files:**
- Modify: `/home/zz/Desktop/tianyi_single_arm/start.bash`
- Modify: `/home/zz/Desktop/tianyi_single_arm/tools/start_arm_bridge.sh`
- Modify: `/home/zz/Desktop/tianyi_single_arm/dependence/smart_pick_and_place_ws/src/pkg_launch/launch/bringup.launch`
- Delete: dual-arm-only bridge/launch source files that are no longer referenced by the single-arm startup path

- [ ] **Step 1: Add a startup static test**

Add a test that reads the startup and launch files and asserts they contain one arm bridge, one Twin service, no right-arm startup command, and no second gripper server.

- [ ] **Step 2: Run the static test before edits**

Run:

```bash
cd /home/zz/Desktop/tianyi_single_arm
python3 -m pytest tests -q -k single_arm_startup
```

Expected: FAIL because `start.bash` still launches left and right bridge/Twin/gripper tabs.

- [ ] **Step 3: Reduce startup to one hardware path**

Make `start.bash` start only the ROS bringup, single arm bridge on `8010`, single Twin on `8020`, one gripper service on the retained local gripper port, and AnyGrasp if required by the existing single-arm skills. Remove checks and process cleanup for `8011`, `8021`, and the second gripper. Change `tools/start_arm_bridge.sh` to accept no side selector and launch only `arm_75_bringup.py`; reject old right-side arguments rather than silently starting a second arm. Make `bringup.launch` load only the RM65 bringup and the single fixed/eye-in-hand camera node required by the retained configuration.

- [ ] **Step 4: Run shell and launch validation**

Run:

```bash
cd /home/zz/Desktop/tianyi_single_arm
bash -n start.bash tools/start_arm_bridge.sh
python3 -m pytest tests -q -k single_arm_startup
rg -n -i 'right_arm|8011|8021|Gripper R|bringup_right|mount_camera_right' start.bash tools/start_arm_bridge.sh dependence/smart_pick_and_place_ws/src/pkg_launch/launch
```

Expected: shell syntax passes; the focused test passes; no active single-arm startup file references the removed second arm.

---

### Task 4: Remove dual-arm runtime routes and reduce Twin/config to one arm

**Files:**
- Modify: `/home/zz/Desktop/tianyi_single_arm/core/arm.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/core/twin_client.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/skills/base.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/skills/pose_execute.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/dependence/twin_inference/twin.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/dependence/twin_inference/sim_world.py`
- Modify: `/home/zz/Desktop/tianyi_single_arm/dependence/twin_inference/robot.py`
- Delete: dual-handover and dual-arm-only skill modules/tests if they are not imported by `skills/__init__.py` or the CLI registry

- [ ] **Step 1: Add a no-dual-runtime test**

Add a test that imports the skill registry and verifies no dual-handover skill is registered, the arm factory exposes one arm, and the Twin request default is the retained single-arm structure.

- [ ] **Step 2: Run the focused test before edits**

Run:

```bash
cd /home/zz/Desktop/tianyi_single_arm
python3 -m pytest tests -q -k no_dual_runtime
```

Expected: FAIL while dual-handover and right-arm routes remain registered.

- [ ] **Step 3: Remove only active dual-arm routes**

Delete dual-arm handover/transfer routing from the skill registry and base skill dispatch. Keep the existing single-arm grasp/place/handover skills if they can run without a second arm. Change Twin request validation and simulation commands to accept only the retained RM65 structure and reject `right_arm`; do not rewrite unrelated planning math. Remove right-arm-only branches from pose execution and configuration lookup.

- [ ] **Step 4: Run focused and syntax checks**

Run:

```bash
cd /home/zz/Desktop/tianyi_single_arm
python3 -m pytest tests -q -k no_dual_runtime
python3 -m compileall -q core skills dependence/twin_inference tools
```

Expected: focused test passes and compilation completes without errors.

---

### Task 5: Validate the cleaned desktop copy without touching hardware

**Files:**
- Modify: `/home/zz/Desktop/tianyi_single_arm/tests/` only if a validation test needs correction

- [ ] **Step 1: Run the complete test suite**

Run:

```bash
cd /home/zz/Desktop/tianyi_single_arm
python3 -m pytest -q
```

Expected: all existing applicable tests pass; tests requiring unavailable hardware are skipped or fail with an explicit connection prerequisite rather than moving the robot.

- [ ] **Step 2: Verify the original project is unchanged**

Run:

```bash
git -C /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World status --short
```

Expected: output exactly matches the pre-existing original-project working-tree state; no desktop-copy edits appear there.

- [ ] **Step 3: Verify the desktop copy's final state**

Run:

```bash
git -C /home/zz/Desktop/tianyi_single_arm status --short --branch
git -C /home/zz/Desktop/tianyi_single_arm diff --stat
git -C /home/zz/Desktop/tianyi_single_arm diff --check
```

Expected: the branch is `tianyi_single_arm`, only the intended single-arm cleanup files are changed/deleted, and `diff --check` is clean.

- [ ] **Step 4: Run only non-motion connectivity checks**

Run:

```bash
python3 - <<'PY'
import socket
for host, port in [('192.168.1.18', 8080), ('127.0.0.1', 8010), ('127.0.0.1', 8020)]:
    s = socket.socket(); s.settimeout(1)
    try:
        s.connect((host, port)); print(host, port, 'reachable')
    except OSError as exc:
        print(host, port, type(exc).__name__)
    finally:
        s.close()
PY
```

Expected: this only tests TCP reachability and sends no motion command. Do not run `start.bash` or any movement skill until the user confirms the robot is powered, clear, and ready.
