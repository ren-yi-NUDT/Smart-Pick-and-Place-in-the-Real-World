#!/usr/bin/env python3
"""PyBullet execution+camera server for the virtual-simulation backend.

Serves on 127.0.0.1:8031. Protocol (mirrors twin.py):
  RECV: raw JSON (no length prefix)
  SEND: 4-byte big-endian length prefix + JSON

Loads both arms (left + right) as independent ``ErdaijiRobot`` bodies. Every
command carries a ``side`` field ("left"/"right") routing it to the matching arm.
"""
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(_THIS_DIR)                                  # robot, utils, p_utils
sys.path.append(os.path.join(_THIS_DIR, "..", ".."))        # core.sim_utils

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
from core.sim_utils import (  # noqa: E402
    deg2rad_list, rad2deg_list, projection_bounds,
    depth_buffer_to_mm, map_gripper_value, parse_mimic_joints,
)

SIM_STEP_DELAY = 1.0 / 240.0
SIM_PORT = 8031

# 双臂相对位姿，取自 dual_arm.urdf 的 base_to_R_base（右臂基座相对左臂）。
# 左臂基座在原点，右臂在 (0.99, -0.72, 0)，绕 Z 转 90°。
_RIGHT_BASE_POS = (0.99, -0.72, 0.0)
_RIGHT_BASE_ORI = (0.0, 0.0, 1.5708)


class SimServer:
    CAM_INTRINSICS = {
        "left": dict(fx=392.268, fy=392.268, cx=325.468, cy=242.282,
                     width=640, height=480, near=0.01, far=3.0),
        "right": dict(fx=392.268, fy=392.268, cx=325.468, cy=242.282,
                      width=640, height=480, near=0.01, far=3.0),
    }

    def __init__(self, vis=True, port=SIM_PORT):
        self.vis = vis
        self.side = "left"
        self._lock = threading.RLock()
        self.urdf_dir = os.path.join(
            os.path.dirname(__file__),
            "../smart_pick_and_place_ws/src/rm_description/urdf",
        )
        self.robots = {}           # side -> ErdaijiRobot
        self.grippers = {}         # side -> gripper dict
        self.camera_link_ids = {}  # side -> link id or None
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
        self._load_arms()
        self._index_cameras()
        self._init_grippers()

    def _load_arms(self):
        self.robots["left"] = self._make_robot(
            os.path.join(self.urdf_dir, "left_arm_bullet.urdf"),
            os.path.join(self.urdf_dir, "robot_config.json"),
            (0.0, 0.0, 0.0), (0, 0, 0),
        )
        self.robots["right"] = self._make_robot(
            os.path.join(self.urdf_dir, "right_arm.urdf"),
            os.path.join(self.urdf_dir, "right_arm_robot_config.json"),
            _RIGHT_BASE_POS, _RIGHT_BASE_ORI,
        )

    def _make_robot(self, urdf_path, config_path, pos, ori):
        robot = ErdaijiRobot(pos, ori,
                             robot_path=urdf_path,
                             config_path=config_path,
                             fixed_robot=True, blocking_mode=False, vis=self.vis)
        robot.load_robot()
        robot.reset_robot()
        return robot

    def _load_scene(self):
        # 场景物体（桌面/容器/待抓物体）留待 Phase 2 按真实工作空间标定。
        # 注意：pybullet 标准 table.urdf 长 1.5m、桌面高 ~0.6m，直接放 [0.45,0,0]
        # 会与机械臂基座重叠且高于抓取位姿（~0.45m），导致碰撞把臂顶出指令位姿。
        # 在标定前不加载桌面，避免污染位姿复现验证。
        pass

    def _index_cameras(self):
        cam_names = {"left": "cam_link_grasp", "right": "R_cam_link_grasp"}
        for side, name in cam_names.items():
            robot = self.robots[side]
            if name in robot.linkName_to_id:
                self.camera_link_ids[side] = robot.linkName_to_id[name]
            else:
                self.camera_link_ids[side] = None

    def _init_grippers(self):
        for side, prefix in [("left", "L_"), ("right", "R_")]:
            self.grippers[side] = self._make_gripper(side, prefix)

    def _make_gripper(self, side, prefix):
        robot = self.robots[side]
        mimic = parse_mimic_joints(robot.robot_path)
        active = prefix + "finger_joint"
        children = {n: m for n, (parent, m) in mimic.items() if parent == active}
        return {
            "active": active,
            "children": children,
            "active_id": robot.jointname_to_id[active],
            "child_ids": {n: robot.jointname_to_id[n] for n in children},
            "close_angle": 0.0,
            "open_angle": 0.8,  # rad；GUI 冒烟时按实际手指张角校准
        }

    def _step(self, n=1):
        with self._lock:
            for _ in range(n):
                for robot in self.robots.values():
                    robot.apply_actions()
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
                threading.Thread(target=self._handle, args=(conn,), daemon=True).start()
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
        if cmd == "get_link_pose":
            return self._get_link_pose(req)
        return {"value": False, "info": {"error": f"unknown cmd {cmd}"}}

    # -- handlers ------------------------------------------------------
    def _arm_struct(self, side):
        return self.robots[side].robot_structs[f"{side}_arm"]

    def _get_joint_state(self, req):
        side = req.get("side", "left")
        with self._lock:
            js_rad = self._arm_struct(side).get_joint_pose()  # radians, 7-list
        return {"value": True, "info": {"js_deg": rad2deg_list(js_rad)}}

    def _execute_trajectory(self, req):
        side = req.get("side", "left")
        trajectory = req.get("trajectory", [])
        if not trajectory:
            return {"value": False, "info": {"error": "empty trajectory"}}
        with self._lock:
            arm = self._arm_struct(side)
            for wp in trajectory:
                js_rad = deg2rad_list(list(wp))
                arm.reset_by_joint_states(js_rad)
                arm.move_joint(js_rad)
                self._step(3)
        return {"value": True, "info": {"n_waypoints": len(trajectory)}}

    def _move_to_pose(self, req):
        side = req.get("side", "left")
        pose = req.get("pose", {})
        js_deg = [pose.get(f"J{i}", 0.0) for i in range(1, 8)]
        with self._lock:
            arm = self._arm_struct(side)
            js_rad = deg2rad_list(js_deg)
            arm.reset_by_joint_states(js_rad)
            arm.move_joint(js_rad)
            self._step(6)
        return {"value": True, "info": {"js_deg": js_deg}}

    def _gripper(self, req):
        side = req.get("side", "left")
        gripper = self.grippers.get(side)
        if gripper is None:
            return {"value": False, "info": {"error": f"no gripper for {side}"}}
        action = req.get("action", "close")
        value = req.get("value", 0 if action == "close" else 1000)
        angle = map_gripper_value(value,
                                  gripper["close_angle"],
                                  gripper["open_angle"])
        rid = self.robots[side].id
        with self._lock:
            p.setJointMotorControl2(rid, gripper["active_id"],
                                    p.POSITION_CONTROL, angle)
            for name, cid in gripper["child_ids"].items():
                mult = gripper["children"][name]
                p.setJointMotorControl2(rid, cid, p.POSITION_CONTROL, angle * mult)
            self._step(6)
        return {"value": True, "info": {"angle": angle, "value": value}}

    def _get_rgbd(self, req):
        side = req.get("side", self.side)
        camera_link_id = self.camera_link_ids.get(side)
        if camera_link_id is None:
            return {"value": False, "info": {"error": f"no camera link for {side}"}}
        intr = self.CAM_INTRINSICS[side]
        robot = self.robots[side]
        with self._lock:
            pos, orn = p.getLinkState(robot.id, camera_link_id)[:2]
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

    def _get_link_pose(self, req):
        side = req.get("side", "left")
        name = req.get("link")
        robot = self.robots[side]
        if name not in robot.linkName_to_id:
            return {"value": False, "info": {"error": f"unknown link {name}"}}
        with self._lock:
            pos, orn = p.getLinkState(robot.id, robot.linkName_to_id[name])[:2]
        return {"value": True, "info": {"pos": list(pos), "orn": list(orn)}}


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
