# 全闭环 PyBullet 虚拟仿真（阶段一：左臂单臂）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增 `SIM_MODE` 开关，把 `arm`/`hand`/`camera` 三个后端切换到 PyBullet 仿真，让 `pick_and_place` 在左臂上跑通「检测→抓取→放置」全闭环，用于调试。

**Architecture:** 新增一个独立 PyBullet 进程 `sim_server.py`（端口 8031），加载 `left_arm_bullet.urdf` + 场景（地面+桌面+物体），通过 socket 提供关节轨迹执行、位姿移动、夹爪开合、RGB-D 渲染、关节角回报。skill 侧新增 `SimArmClient`/`SimGripperClient`/`SimCamera`，接口对齐真实 `ArmClient`/`GripperClient`/`RealSenseCapture`，由 `Config.sim_mode` 在 `base.py`/`arm_side.py` 里分发。

**Tech Stack:** PyBullet, numpy, socket(JSON, 4字节长度前缀), 现有 `ErdaijiRobot`/`sim_world` 孪生代码, conda `anygrasp` 环境。

**关键约定（实现时必须遵守）：**
- 真实 `ArmClient` 接口**全程用度**：`move_to_named_pose`（`robot_config.json` 位姿）与 `execute_trajectory`（twin 返回弧度后 skill 已 `/np.pi*180` 转度，见 `grasp.py:197`）。`SimArmClient` 同样度进；`SimServer` 内部 `*np.pi/180` 转弧度喂 PyBullet。
- 左相机内参（`core/transforms.py`）：`fx=fy=392.268`, `cx=325.468`, `cy=242.282`，图像 640×480。
- 端口：SimServer 8031；AnyGrasp 8030；twin left 8020（均保持）。
- 本阶段只做左臂（`side="left"`）；右臂与双臂留到阶段二。

---

### Task 1: `core/sim_utils.py` 纯逻辑助手 + `Config.sim_mode`

**Files:**
- Create: `core/sim_utils.py`
- Create: `tests/test_sim_utils.py`
- Modify: `core/config.py`

- [ ] **Step 1: 写失败测试**

创建 `tests/test_sim_utils.py`：

```python
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.sim_utils import (
    deg2rad_list, rad2deg_list, projection_bounds,
    depth_buffer_to_mm, map_gripper_value, parse_mimic_joints,
)


def test_deg2rad_list():
    out = deg2rad_list([0.0, 180.0, 90.0])
    assert np.allclose(out, [0.0, np.pi, np.pi / 2], atol=1e-9)


def test_rad2deg_list():
    out = rad2deg_list([0.0, np.pi, np.pi / 2])
    assert np.allclose(out, [0.0, 180.0, 90.0], atol=1e-6)


def test_projection_bounds_left_camera():
    fx = fy = 392.268
    cx, cy = 325.468, 242.282
    left, right, bottom, top = projection_bounds(fx, fy, cx, cy, 640, 480, near=0.01)
    assert left < 0 < right
    assert bottom < 0 < top
    assert np.isclose(left, -cx * 0.01 / fx)
    assert np.isclose(top, (480 - cy) * 0.01 / fy)


def test_depth_buffer_to_mm_roundtrip():
    near, far = 0.01, 3.0
    buf = np.array([0.0, 0.5, 1.0], dtype=np.float32)  # 0=near, 1=far
    mm = depth_buffer_to_mm(buf, near, far)
    assert mm.dtype == np.uint16
    assert mm[0] == 10  # near=0.01m -> 10mm
    assert mm[2] == 3000  # far=3m -> 3000mm


def test_map_gripper_value():
    assert map_gripper_value(0, close=0.0, open=0.8) == 0.0
    assert np.isclose(map_gripper_value(1000, close=0.0, open=0.8), 0.8)
    assert np.isclose(map_gripper_value(500, close=0.0, open=0.8), 0.4)


def test_parse_mimic_joints():
    urdf = """<?xml version="1.0"?>
<robot name="t">
  <joint name="L_finger_joint" type="revolute"><parent link="b"/><child link="f"/></joint>
  <joint name="L_left_inner_knuckle_joint" type="revolute">
    <parent link="b"/><child link="k"/>
    <mimic joint="L_finger_joint" multiplier="1" offset="0"/>
  </joint>
  <joint name="L_left_inner_finger_joint" type="revolute">
    <parent link="k"/><child link="f2"/>
    <mimic joint="L_finger_joint" multiplier="-1" offset="0"/>
  </joint>
</robot>"""
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(urdf)
        path = f.name
    mapping = parse_mimic_joints(path)
    os.unlink(path)
    assert mapping["L_left_inner_knuckle_joint"] == ("L_finger_joint", 1.0)
    assert mapping["L_left_inner_finger_joint"] == ("L_finger_joint", -1.0)
    assert "L_finger_joint" not in mapping
```

- [ ] **Step 2: 运行确认失败**

Run: `cd /home/zz/Code/Smart-Pick-and-Place-in-the-Real-World && python3 tests/test_sim_utils.py`
Expected: FAIL（`ModuleNotFoundError: No module named 'core.sim_utils'`）

- [ ] **Step 3: 实现 `core/sim_utils.py`**

```python
"""Pure-logic helpers for the PyBullet virtual-simulation backend.

Kept free of pybullet/rospy imports so they can be unit-tested in isolation.
"""
import os
import xml.etree.ElementTree as ET

import numpy as np

DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi


def deg2rad_list(deg_values):
    return [float(v) * DEG2RAD for v in deg_values]


def rad2deg_list(rad_values):
    return [float(v) * RAD2DEG for v in rad_values]


def projection_bounds(fx, fy, cx, cy, width, height, near):
    """OpenGL projection bounds matching a pinhole camera's intrinsics."""
    left = -cx * near / fx
    right = (width - cx) * near / fx
    bottom = -cy * near / fy
    top = (height - cy) * near / fy
    return left, right, bottom, top


def depth_buffer_to_mm(depth_buffer, near, far):
    """Convert PyBullet tiny-renderer depth buffer ([0,1]) to uint16 mm.

    depth_buffer==0 means the near plane, 1 means the far plane.
    """
    buf = np.asarray(depth_buffer, dtype=np.float32)
    depth_m = far * near / (far - (far - near) * buf)
    return (depth_m * 1000.0).astype(np.uint16)


def map_gripper_value(value_0_1000, close=0.0, open=0.8):
    """Map Robotiq 85 command value (0=close, 1000=open) to finger angle."""
    frac = float(value_0_1000) / 1000.0
    return close + (open - close) * frac


def parse_mimic_joints(urdf_path):
    """Return {mimic_child_name: (parent_joint_name, multiplier)} from a URDF."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    mapping = {}
    for joint in root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        name = joint.get("name")
        parent = mimic.get("joint")
        multiplier = float(mimic.get("multiplier", "1"))
        mapping[name] = (parent, multiplier)
    return mapping
```

- [ ] **Step 4: 运行确认通过**

Run: `python3 tests/test_sim_utils.py`
Expected: PASS（脚本内所有 `assert` 通过，无输出即成功）

- [ ] **Step 5: 给 `core/config.py` 加 `sim_mode`**

在 `Config` 类的 `__init__` 末尾（`self.reload()` 之后）无改动，改为新增 property。在 `reload()` 后紧跟添加：

```python
    @property
    def sim_mode(self) -> bool:
        """True when running against the PyBullet sim backend.

        Priority: SIM_MODE env var (explicit 1/true/yes/on or 0/false/no/off)
        overrides robot_config.json ``shared.sim_mode``.
        """
        env = os.environ.get("SIM_MODE", "").strip().lower()
        if env in ("1", "true", "yes", "on"):
            return True
        if env in ("0", "false", "no", "off"):
            return False
        return bool(self._shared.get("sim_mode", False))
```

- [ ] **Step 6: 验证 + 提交**

Run: `python3 -c "import os; os.environ['SIM_MODE']='1'; from core.config import Config; print(Config().sim_mode)"` Expected: `True`

```bash
git add core/sim_utils.py tests/test_sim_utils.py core/config.py
git commit -m "feat: add sim_utils helpers and Config.sim_mode flag"
```

---

### Task 2: SimServer 骨架（加载左臂 URDF + 场景 + socket）

**Files:**
- Create: `dependence/twin_inference/sim_server.py`

- [ ] **Step 1: 写 SimServer 骨架**

创建 `dependence/twin_inference/sim_server.py`（参照 `twin.py` 的 `TwinTest2`，但加载单臂 + 场景 + 仅做执行/相机，不做 IK）：

```python
#!/usr/bin/env python3
"""PyBullet execution+camera server for the virtual-simulation backend.

Serves on 127.0.0.1:8031. Protocol (mirrors twin.py):
  RECV: raw JSON (no length prefix)
  SEND: 4-byte big-endian length prefix + JSON
"""
import os
import sys

sys.path.append(os.path.dirname(__file__))

import argparse
import base64
import io
import json
import socket
import struct
import threading

import numpy as np
import pybullet as p
import pybullet_data
import rospy
from termcolor import cprint

from robot import ErdaijiRobot
from core.sim_utils import (  # noqa: E402  (sys.path above)
    deg2rad_list, rad2deg_list, projection_bounds,
    depth_buffer_to_mm, map_gripper_value, parse_mimic_joints,
)

SIM_STEP_DELAY = 1.0 / 240.0
SIM_PORT = 8031


class SimServer:
    def __init__(self, vis=True, port=SIM_PORT):
        self.vis = vis
        self.side = "left"
        self.urdf_dir = os.path.join(
            os.path.dirname(__file__),
            "../smart_pick_and_place_ws/src/rm_description/urdf",
        )
        self.urdf_path = os.path.join(self.urdf_dir, "left_arm_bullet.urdf")
        self.robot = None
        self.gripper = None
        self.camera_link_id = None
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(("127.0.0.1", port))
        self.server_socket.listen(5)
        threading.Thread(target=self._serve, daemon=True).start()
        self._setup()

    # -- world ---------------------------------------------------------
    def _setup(self):
        self.physics_client = p.connect(p.GUI if self.vis else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        p.resetDebugVisualizerCamera(cameraDistance=1.8, cameraYaw=50,
                                     cameraPitch=-35, cameraTargetPosition=[0.4, 0, 0.3])
        p.loadURDF("plane.urdf")
        self._load_scene()
        self.robot = ErdaijiRobot((0.0, 0.0, 0.0), (0, 0, 0),
                                  robot_path=self.urdf_path,
                                  config_path=os.path.join(self.urdf_dir, "robot_config.json"),
                                  fixed_robot=True, vis=self.vis)
        self.robot.load_robot()
        self._index_camera()

    def _load_scene(self):
        # 桌面 + 一个可抓物体 + 一个容器（占位，Task 9 再充实）
        table_pos = [0.45, 0.0, 0.0]
        p.loadURDF("table/table.urdf", table_pos, useFixedBase=True)

    def _index_camera(self):
        # 相机 link 名（Task 5 添加到 URDF 后生效）
        name = "cam_link_grasp"
        if name in self.robot.linkName_to_id:
            self.camera_link_id = self.robot.linkName_to_id[name]
        else:
            self.camera_link_id = None

    def _step(self, n=1):
        for _ in range(n):
            self.robot.apply_actions()
            p.stepSimulation()
            rospy.sleep(SIM_STEP_DELAY)

    # -- socket --------------------------------------------------------
    def _serve(self):
        while not rospy.is_shutdown():
            try:
                self.server_socket.settimeout(1.0)
                try:
                    conn, _ = self.server_socket.accept()
                except socket.timeout:
                    continue
                self._handle(conn)
            except Exception as e:  # noqa: BLE001
                cprint(f"[SimServer] serve error: {e}", "red")
                rospy.sleep(1)

    def _handle(self, conn):
        with conn:
            while not rospy.is_shutdown():
                try:
                    data = conn.recv(65536)
                    if not data:
                        break
                    resp = self.dispatch(json.loads(data.decode("utf-8")))
                    payload = json.dumps(resp).encode("utf-8")
                    conn.sendall(struct.pack(">I", len(payload)))
                    conn.sendall(payload)
                except Exception as e:  # noqa: BLE001
                    cprint(f"[SimServer] client error: {e}", "red")
                    break

    def dispatch(self, req):
        cmd = req.get("cmd")
        if cmd == "reset":
            return {"value": True, "info": {}}
        if cmd == "get_joint_state":
            return self._get_joint_state(req)
        if cmd == "execute_trajectory":
            return self._execute_trajectory(req)
        if cmd == "move_to_pose":
            return self._move_to_pose(req)
        if cmd == "gripper":
            return self._gripper(req)
        if cmd == "get_rgbd":
            return self._get_rgbd(req)
        return {"value": False, "info": {"error": f"unknown cmd {cmd}"}}

    # -- handlers（Task 3/4/5 逐个实现）-------------------------------
    def _get_joint_state(self, req):
        return {"value": False, "info": {"error": "not implemented"}}

    def _execute_trajectory(self, req):
        return {"value": False, "info": {"error": "not implemented"}}

    def _move_to_pose(self, req):
        return {"value": False, "info": {"error": "not implemented"}}

    def _gripper(self, req):
        return {"value": False, "info": {"error": "not implemented"}}

    def _get_rgbd(self, req):
        return {"value": False, "info": {"error": "not implemented"}}


if __name__ == "__main__":
    argv = rospy.myargv()
    parser = argparse.ArgumentParser()
    parser.add_argument("--novis", action="store_true")
    parser.add_argument("--port", type=int, default=SIM_PORT)
    args, _ = parser.parse_known_args(argv[1:])
    rospy.init_node("sim_server", anonymous=True)
    server = SimServer(vis=not args.novis, port=args.port)
    while not rospy.is_shutdown():
        server._step()
        rospy.sleep(SIM_STEP_DELAY)
```

- [ ] **Step 2: 语法检查**

Run: `cd dependence/twin_inference && python3 -m py_compile sim_server.py`
Expected: 无输出（编译通过）

- [ ] **Step 3: 提交**

```bash
git add dependence/twin_inference/sim_server.py
git commit -m "feat: add SimServer skeleton (load left arm URDF + scene + socket)"
```

> 注意：本 Task 的 `_get_joint_state` 等 handler 是桩，故意先提交骨架；Task 3/4/5 逐个实现。骨架里 `from core.sim_utils import ...` 依赖 Task 1 已提交。

---

### Task 3: SimServer 机械臂执行（get_joint_state / execute_trajectory / move_to_pose）

**Files:**
- Modify: `dependence/twin_inference/sim_server.py`

- [ ] **Step 1: 实现 handler**

把 `sim_server.py` 里三个 handler 替换为：

```python
    def _arm_struct(self):
        return self.robot.robot_structs["left_arm"]

    def _get_joint_state(self, req):
        js_rad = self._arm_struct().get_joint_pose()  # radians, 7-list
        return {"value": True, "info": {"js_deg": rad2deg_list(js_rad)}}

    def _execute_trajectory(self, req):
        trajectory = req.get("trajectory", [])
        if not trajectory:
            return {"value": False, "info": {"error": "empty trajectory"}}
        arm = self._arm_struct()
        for wp in trajectory:
            js_rad = deg2rad_list(list(wp))
            arm.reset_by_joint_states(js_rad)
            arm.move_joint(js_rad)
            self._step(3)
        return {"value": True, "info": {"n_waypoints": len(trajectory)}}

    def _move_to_pose(self, req):
        pose = req.get("pose", {})
        js_deg = [pose.get(f"J{i}", 0.0) for i in range(1, 8)]
        arm = self._arm_struct()
        js_rad = deg2rad_list(js_deg)
        arm.reset_by_joint_states(js_rad)
        arm.move_joint(js_rad)
        self._step(6)
        return {"value": True, "info": {"js_deg": js_deg}}
```

- [ ] **Step 2: 语法检查**

Run: `cd dependence/twin_inference && python3 -m py_compile sim_server.py`
Expected: 无输出

- [ ] **Step 3: 手动 GUI 验证（可选，Task 8 一起做）**

启动后发 `get_joint_state` 应返回 7 个度值；`move_to_pose` 到 grasp1（从 `robot_config.json` 取）应看到左臂动。

- [ ] **Step 4: 提交**

```bash
git add dependence/twin_inference/sim_server.py
git commit -m "feat: implement SimServer arm execution (trajectory/pose/joint_state)"
```

---

### Task 4: SimServer 夹爪（mimic 关节驱动）

**Files:**
- Modify: `dependence/twin_inference/sim_server.py`

- [ ] **Step 1: 在 `_setup()` 末尾初始化夹爪控制器**

在 `_setup()` 的 `self._index_camera()` 之后加：

```python
        self._init_gripper()
```

新增方法：

```python
    def _init_gripper(self):
        mimic = parse_mimic_joints(self.urdf_path)
        active = "L_finger_joint"
        children = {n: m for n, (parent, m) in mimic.items() if parent == active}
        self.gripper = {
            "active": active,
            "children": children,
            "active_id": self.robot.jointname_to_id[active],
            "child_ids": {n: self.robot.jointname_to_id[n] for n in children},
            "close_angle": 0.0,
            "open_angle": 0.8,  # rad，GUI 下校准
        }

    def _gripper(self, req):
        if self.gripper is None:
            self._init_gripper()
        action = req.get("action", "close")
        value = req.get("value", 0 if action == "close" else 1000)
        angle = map_gripper_value(value,
                                  self.gripper["close_angle"],
                                  self.gripper["open_angle"])
        rid = self.robot.id
        p.setJointMotorControl2(rid, self.gripper["active_id"],
                                p.POSITION_CONTROL, angle)
        for cid, mult in self.gripper["child_ids"].items():
            p.setJointMotorControl2(rid, cid, p.POSITION_CONTROL, angle * mult)
        self._step(6)
        return {"value": True, "info": {"angle": angle, "value": value}}
```

- [ ] **Step 2: 语法检查**

Run: `cd dependence/twin_inference && python3 -m py_compile sim_server.py`
Expected: 无输出

- [ ] **Step 3: 提交**

```bash
git add dependence/twin_inference/sim_server.py
git commit -m "feat: implement SimServer gripper open/close via mimic joints"
```

> `open_angle=0.8` 是占位，GUI 下需校准手指张角；若 `L_finger_joint` 的 0 位是「张开」而非「闭合」，交换 open/close 角度即可。

---

### Task 5: SimServer RGB-D 相机（URDF 相机 link + 渲染）

**Files:**
- Modify: `dependence/smart_pick_and_place_ws/src/rm_description/urdf/left_arm_bullet.urdf`
- Modify: `dependence/twin_inference/sim_server.py`

- [ ] **Step 1: 给 `left_arm_bullet.urdf` 加相机 link**

在 `</robot>` 前插入（挂到 `Link7`，偏移为占位，需 GUI 校准朝向桌子）：

```xml
  <joint name="cam_link_grasp_joint" type="fixed">
    <parent link="Link7"/>
    <child link="cam_link_grasp"/>
    <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
  </joint>
  <link name="cam_link_grasp"/>
```

> 说明：`cam_link_grasp` 是空 link（无几何），只提供坐标系。偏移 `xyz/rpy` 先设 0，Task 8 冒烟时按「arm 在 grasp1 时相机能看到桌面物体」调。

- [ ] **Step 2: 实现 `_get_rgbd`**

在 `sim_server.py` 里新增相机常量与 handler（`_index_camera` 已存在）：

```python
    CAM_INTRINSICS = {
        "left": dict(fx=392.268, fy=392.268, cx=325.468, cy=242.282,
                     width=640, height=480, near=0.01, far=3.0),
    }

    def _get_rgbd(self, req):
        if self.camera_link_id is None:
            return {"value": False, "info": {"error": "no camera link"}}
        intr = self.CAM_INTRINSICS[self.side]
        pos, orn = p.getLinkState(self.robot.id, self.camera_link_id)[:2]
        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        forward = rot @ np.array([0, 0, 1.0])
        up = rot @ np.array([0, -1.0, 0])
        view = p.computeViewMatrix(pos, pos + forward, up)
        left, right, bottom, top = projection_bounds(
            intr["fx"], intr["fy"], intr["cx"], intr["cy"],
            intr["width"], intr["height"], intr["near"],
        )
        proj = p.computeProjectionMatrix(left, right, bottom, top,
                                         intr["near"], intr["far"])
        w, h, rgb, depth, _ = p.getCameraImage(
            intr["width"], intr["height"], view, proj,
            renderer=p.ER_TINY_RENDERER,
        )
        rgb_arr = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
        depth_mm = depth_buffer_to_mm(np.asarray(depth, dtype=np.float32),
                                      intr["near"], intr["far"])
        buf = io.BytesIO()
        from PIL import Image
        Image.fromarray(rgb_arr).save(buf, format="PNG")
        return {
            "value": True,
            "info": {
                "width": w, "height": h,
                "rgb_b64": base64.b64encode(buf.getvalue()).decode("ascii"),
                "depth_b64": base64.b64encode(depth_mm.tobytes()).decode("ascii"),
            },
        }
```

- [ ] **Step 3: 语法检查**

Run: `cd dependence/twin_inference && python3 -m py_compile sim_server.py`
Expected: 无输出

- [ ] **Step 4: 提交**

```bash
git add dependence/twin_inference/sim_server.py dependence/smart_pick_and_place_ws/src/rm_description/urdf/left_arm_bullet.urdf
git commit -m "feat: implement SimServer RGB-D camera rendering + camera link"
```

---

### Task 6: 仿真客户端（SimArmClient / SimGripperClient / SimCamera）

**Files:**
- Create: `core/sim_arm.py`
- Create: `core/sim_gripper.py`
- Create: `core/sim_camera.py`
- Test: `tests/test_sim_clients.py`

- [ ] **Step 1: 写失败测试（客户端协议）**

创建 `tests/test_sim_clients.py`：

```python
import json
import os
import socket
import struct
import sys
import threading

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.sim_arm import SimArmClient
from core.sim_gripper import SimGripperClient

SIM_PORT = 18331


def _recv_frame(conn):
    hdr = b""
    while len(hdr) < 4:
        hdr += conn.recv(4 - len(hdr))
    n = struct.unpack(">I", hdr)[0]
    data = b""
    while len(data) < n:
        data += conn.recv(n - len(data))
    return json.loads(data.decode("utf-8"))


class FakeSimServer:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", SIM_PORT))
        self.sock.listen(1)
        self.received = []
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        conn, _ = self.sock.accept()
        with conn:
            while True:
                data = conn.recv(65536)
                if not data:
                    break
                req = json.loads(data.decode("utf-8"))
                self.received.append(req)
                payload = json.dumps({"value": True, "info": {}}).encode()
                conn.sendall(struct.pack(">I", len(payload)) + payload)


def test_sim_arm_execute_trajectory_sends_degrees():
    server = FakeSimServer()
    c = SimArmClient(host="127.0.0.1", port=SIM_PORT)
    assert c.connect() is True
    c.execute_trajectory([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]], speed=20)
    req = server.received[-1]
    assert req["cmd"] == "execute_trajectory"
    assert req["trajectory"] == [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]]


def test_sim_gripper_open_sends_value_1000():
    server = FakeSimServer()
    c = SimGripperClient(host="127.0.0.1", port=SIM_PORT, src="/left_gripper/movement_control")
    c.connect()
    c.open()
    req = server.received[-1]
    assert req["cmd"] == "gripper"
    assert req["action"] == "open"
    assert req["value"] == 1000
```

- [ ] **Step 2: 运行确认失败**

Run: `python3 tests/test_sim_clients.py`
Expected: FAIL（`ModuleNotFoundError`）

- [ ] **Step 3: 实现三个客户端**

创建 `core/sim_arm.py`：

```python
"""Arm client that routes joint commands to the PyBullet SimServer (port 8031).

Interface mirrors core.arm.ArmClient: degrees everywhere.
"""
import json
import socket
import struct

from termcolor import cprint


class SimArmClient:
    def __init__(self, host="127.0.0.1", port=8031, side="left"):
        self.host = host
        self.port = port
        self.side = side
        self.sock = None
        self._cmds = []

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            return True
        except Exception as e:  # noqa: BLE001
            cprint(f"[SimArmClient] connect failed: {e}", "red")
            return False

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _send(self, data):
        self.sock.sendall(json.dumps(data).encode("utf-8"))
        hdr = b""
        while len(hdr) < 4:
            hdr += self.sock.recv(4 - len(hdr))
        n = struct.unpack(">I", hdr)[0]
        body = b""
        while len(body) < n:
            body += self.sock.recv(min(65536, n - len(body)))
        return json.loads(body.decode("utf-8"))

    # -- ArmClient-compatible surface --------------------------------
    def reset_cmd(self):
        self._cmds = []

    def start_cmd(self):
        self._cmds.append({"type": "start", "act": []})

    def add_js_cmd(self, joint_dict, speed=5, block=True):
        self._cmds.append({"type": "js", "act": joint_dict,
                           "speed": speed, "block": block})

    def add_ee_cmd(self, ee_trajectory, speed=5, block=True):
        self._cmds.append({"type": "ee", "act": ee_trajectory,
                           "speed": speed, "block": block})

    def send_cmds(self):
        # 把累积的 js 命令打平成一条 execute_trajectory
        traj = []
        for c in self._cmds:
            if c["type"] == "js":
                act = c["act"]
                traj.append([act.get(f"J{i}", 0.0) for i in range(1, 8)])
        self.reset_cmd()
        return self._send({"cmd": "execute_trajectory",
                           "side": self.side, "trajectory": traj})

    def move_to_named_pose(self, pose_dict, speed=30):
        self._send({"cmd": "move_to_pose", "side": self.side,
                    "pose": pose_dict, "speed": speed})
        return True

    def execute_trajectory(self, trajectory, speed=20):
        self._send({"cmd": "execute_trajectory", "side": self.side,
                    "trajectory": list(trajectory), "speed": speed})
        return True
```

创建 `core/sim_gripper.py`：

```python
"""Gripper client routing open/close to the PyBullet SimServer."""
import json
import socket
import struct

from termcolor import cprint


class SimGripperClient:
    def __init__(self, host="127.0.0.1", port=8031, src="/left_gripper/movement_control"):
        self.host = host
        self.port = port
        self.src = src
        self.side = "left" if "left" in src else "right"
        self.sock = None
        self._pos = 1000  # 内部状态：1000=open

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            return True
        except Exception as e:  # noqa: BLE001
            cprint(f"[SimGripperClient] connect failed: {e}", "red")
            return False

    def close_connection(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _send(self, data):
        self.sock.sendall(json.dumps(data).encode("utf-8"))
        hdr = b""
        while len(hdr) < 4:
            hdr += self.sock.recv(4 - len(hdr))
        n = struct.unpack(">I", hdr)[0]
        body = b""
        while len(body) < n:
            body += self.sock.recv(min(65536, n - len(body)))
        return json.loads(body.decode("utf-8"))

    def open(self, force=None, speed=None):
        self._pos = 1000
        return self._send({"cmd": "gripper", "side": self.side,
                           "action": "open", "value": 1000})

    def close(self, force=None, speed=None, soft=False):
        self._pos = 0
        return self._send({"cmd": "gripper", "side": self.side,
                           "action": "close", "value": 0})

    def get_state(self):
        return self._send({"cmd": "get_joint_state", "side": self.side})

    def is_grasping(self):
        # 简化：夹爪处于闭合态即视为抓握（MVP；真实 gOBJ 检测留阶段二）
        return self._pos < 500

    def is_fully_open(self):
        return self._pos >= 950

    def get_finger_deviation(self):
        return 1000 - self._pos
```

创建 `core/sim_camera.py`：

```python
"""RGB-D camera backed by the PyBullet SimServer's get_rgbd."""
import base64
import json
import socket
import struct

import numpy as np
from termcolor import cprint


class SimCamera:
    def __init__(self, width=640, height=480, fps=30, save_path="", serial="",
                 host="127.0.0.1", port=8031, side="left"):
        self.width = width
        self.height = height
        self.save_path = save_path
        self.serial = serial
        self.host = host
        self.port = port
        self.side = side
        self.sock = None

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            return True
        except Exception as e:  # noqa: BLE001
            cprint(f"[SimCamera] connect failed: {e}", "red")
            return False

    def get_rgbd(self):
        self.sock.sendall(json.dumps({"cmd": "get_rgbd", "side": self.side}).encode("utf-8"))
        hdr = b""
        while len(hdr) < 4:
            hdr += self.sock.recv(4 - len(hdr))
        n = struct.unpack(">I", hdr)[0]
        body = b""
        while len(body) < n:
            body += self.sock.recv(min(1 << 20, n - len(body)))
        info = json.loads(body.decode("utf-8"))["info"]
        from PIL import Image
        import io
        rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(info["rgb_b64"])))).copy()
        depth = np.frombuffer(base64.b64decode(info["depth_b64"]), dtype=np.uint16)
        depth = depth.reshape(info["height"], info["width"])
        return rgb, depth

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
```

- [ ] **Step 4: 运行确认通过**

Run: `python3 tests/test_sim_clients.py`
Expected: PASS（两个断言通过，脚本无异常退出）

- [ ] **Step 5: 提交**

```bash
git add core/sim_arm.py core/sim_gripper.py core/sim_camera.py tests/test_sim_clients.py
git commit -m "feat: add SimArmClient/SimGripperClient/SimCamera routing to SimServer"
```

---

### Task 7: base.py / arm_side.py 的 sim_mode 分发 + TF 旁路

**Files:**
- Modify: `skills/base.py`
- Modify: `core/arm_side.py`

- [ ] **Step 1: `core/arm_side.py` 分发 arm/hand**

在 `ArmSide` 类中，把 `arm` property 与 `hand` property 改为按 `sim_mode` 分发。`ArmSide.__init__` 增加 `sim_mode` 参数：

```python
    def __init__(self, side, arm_config, host="127.0.0.1", sim_mode=False):
        ...
        self._sim_mode = sim_mode

    @property
    def arm(self):
        if self._arm is None:
            if self._sim_mode:
                from core.sim_arm import SimArmClient
                self._arm = SimArmClient(host=self._host, port=8031, side=self.side)
                self._arm.connect()
            else:
                port = self._arm_config["arm_port"]
                self._arm = ArmClient(host=self._host, port=port)
                self._arm.connect()
        return self._arm

    @property
    def hand(self):
        if self._hand is None:
            if self._sim_mode:
                from core.sim_gripper import SimGripperClient
                self._hand = SimGripperClient(host=self._host, port=8031,
                                              src=f"/{self.side}_gripper/movement_control")
                self._hand.connect()
            else:
                hand_type = self._arm_config.get("hand_type", "dexterous")
                port = self._arm_config["hand_port"]
                if hand_type == "gripper":
                    src = f"/{self.side}_gripper/movement_control"
                    self._hand = GripperClient(host=self._host, port=port, src=src)
                else:
                    self._hand = HandClient(host=self._host, port=port)
                self._hand.connect()
        return self._hand
```

同时把 `DualArmSkill.__init__`（`skills/base.py`）里创建 `ArmSide` 的两行改为传入 `sim_mode=self.config.sim_mode`。

- [ ] **Step 2: `skills/base.py` 分发 `arm`/`hand`/`camera`**

`Skill.arm` property 改为：

```python
    @property
    def arm(self):
        if self._arm is None:
            if self.config.sim_mode:
                from core.sim_arm import SimArmClient
                host = self.config.shared.get("host", "127.0.0.1")
                self._arm = SimArmClient(host, 8031, side="left")
                self._arm.connect()
            else:
                from core.arm import ArmClient
                # ... 原有逻辑 ...
                self._arm = client
        return self._arm
```

`Skill.hand` property 在 `sim_mode` 时返回 `SimGripperClient`（port 8031, src `/left_gripper/movement_control`）。`Skill._make_camera` 在 `sim_mode` 时返回 `SimCamera`（host/port/side="left"）。

- [ ] **Step 3: TF 旁路（`save_current_transformation`）**

`sim_mode` 下 ROS TF 不可用，改用 SimServer 报告的位姿。在 `skills/base.py` 给 `Skill` 加一个辅助方法并在 `save_current_transformation` 里分支：

```python
    def save_current_transformation(self):
        if self.config.sim_mode:
            self._save_transforms_from_sim()
            return
        # ... 原有 ROS TF 逻辑 ...

    def _save_transforms_from_sim(self):
        # 单臂 URDF base 在原点：base_link 位姿为单位阵
        import numpy as np
        cam = self.camera  # SimCamera，尚未用，直接取 SimServer 位姿
        # 用 SimArmClient 的 _send 取 get_link_pose（需 SimServer 暴露，见下）
        link = self.arm._send({"cmd": "get_link_pose", "side": "left",
                               "link": "cam_link_grasp"})
        pos, orn = link["info"]["pos"], link["info"]["orn"]
        from scipy.spatial.transform import Rotation as R
        self.T_base_to_cam = np.eye(4)
        self.T_base_to_cam[:3, :3] = R.from_quat(orn).as_matrix()
        self.T_base_to_cam[:3, 3] = pos
        # 末端/手效应器到 arm_end_link：左臂 hand_effector=L_gripper_endeffector, arm_end=Link7
        hand = self.arm._send({"cmd": "get_link_pose", "side": "left",
                               "link": "L_gripper_endeffector"})["info"]
        end = self.arm._send({"cmd": "get_link_pose", "side": "left",
                              "link": "Link7"})["info"]
        T_end = np.eye(4); T_end[:3, :3] = R.from_quat(end["orn"]).as_matrix(); T_end[:3, 3] = end["pos"]
        T_hand = np.eye(4); T_hand[:3, :3] = R.from_quat(hand["orn"]).as_matrix(); T_hand[:3, 3] = hand["pos"]
        self.T_hand_effector_to_arm_endlink = np.linalg.inv(T_end) @ T_hand
```

- [ ] **Step 4: SimServer 增加 `get_link_pose`**

在 `sim_server.py` 的 `dispatch` 加分支，并实现：

```python
        if cmd == "get_link_pose":
            return self._get_link_pose(req)

    def _get_link_pose(self, req):
        name = req.get("link")
        if name not in self.robot.linkName_to_id:
            return {"value": False, "info": {"error": f"unknown link {name}"}}
        pos, orn = p.getLinkState(self.robot.id, self.robot.linkName_to_id[name])[:2]
        return {"value": True, "info": {"pos": list(pos), "orn": list(orn)}}
```

- [ ] **Step 5: 语法检查 + 提交**

Run: `python3 -m py_compile skills/base.py core/arm_side.py dependence/twin_inference/sim_server.py`
Expected: 无输出

```bash
git add skills/base.py core/arm_side.py dependence/twin_inference/sim_server.py
git commit -m "feat: dispatch arm/hand/camera to sim clients under sim_mode + TF bypass"
```

---

### Task 8: 端到端冒烟 + start_sim.bash

**Files:**
- Create: `start_sim.bash`

- [ ] **Step 1: 写 `start_sim.bash`**

```bash
#!/bin/bash
set -e
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

xfce4-terminal \
  --tab --title="SimServer (:8031)" \
  --working-directory="$PROJECT_ROOT/dependence/twin_inference" \
  --command="bash -ic 'source /opt/ros/noetic/setup.bash && conda activate anygrasp && python3 sim_server.py; exec bash'" \
  --tab --title="Twin IK left (:8020)" \
  --working-directory="$PROJECT_ROOT/dependence/twin_inference" \
  --command="bash -ic 'source /opt/ros/noetic/setup.bash && conda activate anygrasp && python3 twin.py --side left; exec bash'" \
  --tab --title="AnyGrasp Server (:8030)" \
  --working-directory="$PROJECT_ROOT/dependence/anygrasp_server" \
  --command="bash -ic 'source /home/zz/anaconda3/etc/profile.d/conda.sh && conda activate anygrasp && export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/zz/anaconda3/envs/anygrasp/lib/python3.9/site-packages/nvidia/cudnn/lib && python3 anygrasp_server.py; exec bash'"

echo "Started SimServer(8031) + Twin IK left(8020) + AnyGrasp(8030)"
echo "然后在另一个终端: SIM_MODE=1 python run_skill.py pick_and_place <<< '{\"object\":\"cup\",\"container\":\"bowl\"}'"
```

- [ ] **Step 2: 冒烟测试（人工，GUI）**

1. `chmod +x start_sim.bash && ./start_sim.bash`，确认三个 tab 起来、SimServer GUI 里出现左臂 + 桌面。
2. 发关节状态：`python3 -c "from core.sim_arm import SimArmClient; c=SimArmClient(); c.connect(); import json; print(json.dumps(c._send({'cmd':'get_joint_state','side':'left'})))"` → 应返回 7 个度值。
3. 移动到位姿：用 `robot_config.json` 的 grasp1 发 `move_to_pose`，确认 GUI 左臂动。
4. 相机：`python3 -c "from core.sim_camera import SimCamera; c=SimCamera(); c.connect(); rgb,d=c.get_rgbd(); print(rgb.shape, d.shape, d.dtype)"` → `(480,640,3) (480,640) uint16`；保存 `rgb` 看图确认能拍到桌面。
5. **校准相机 link 与场景**：若相机拍不到桌面，回到 Task 5 调 `cam_link_grasp_joint` 的 `xyz/rpy`；加一个物体（`p.loadURDF("cube.urdf", ...)` 或 `p.createCollisionShape`）到桌面，确保 YOLO 能检出目标类。
6. 端到端：`SIM_MODE=1 python run_skill.py pick_and_place <<< '{"object":"cup","container":"bowl"}'`，观察检测→抓取→放置全程。

- [ ] **Step 3: 提交**

```bash
git add start_sim.bash
git commit -m "feat: add start_sim.bash for headless sim launch"
```

---

## Self-Review 记录

- **Spec 覆盖**：spec 的 4.1(SimServer) → Task 2/3/4/5；4.2(客户端) → Task 6；4.3(分发) → Task 7；4.4(启动) → Task 8；5.1(内参) → Task 1/5；5.2(TF 旁路) → Task 7；5.4(单位) → Task 1/3；5.5(mimic) → Task 4。阶段二（右臂/双臂/真值抓取兜底）不在本计划内，另开计划。
- **占位符**：无 TBD/TODO；`open_angle=0.8`、相机 `xyz/rpy=0` 已显式标注为「GUI 校准」而非留空。
- **类型一致**：`execute_trajectory`/`move_to_pose` 度进（客户端与 server 一致）；`_send` 返回 dict；`SimGripperClient.side` 从 `src` 推导；`SimCamera.get_rgbd` 返回 `(H,W,3) uint8` 与 `(H,W) uint16`，对齐 `RealSenseCapture`。

## 阶段二（后续，另开计划）

右臂（对称）、双臂 handover 物理仿真、`is_grasping` 真实物体接触判定、真值抓取兜底（绕过 AnyGrasp）、场景多物体/杂波、`fetch_from_user` 验证。
