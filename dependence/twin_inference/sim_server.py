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
import copy
import io
import json
import socket
import struct
import threading
from datetime import datetime

import numpy as np
import pybullet as p
import pybullet_data
import rospy
from scipy.spatial.transform import Rotation
from termcolor import cprint

from robot import ErdaijiRobot
from core.config import Config
from core.sim_utils import (  # noqa: E402
    deg2rad_list, rad2deg_list, projection_bounds,
    depth_buffer_to_mm, map_gripper_value, parse_mimic_joints,
)

SIM_STEP_DELAY = 1.0 / 240.0
SIM_PORT = 8031
ARM_MAX_VEL = 1.2  # rad/s; caps arm joint velocity so motion is smooth, not teleport

# 双臂相对位姿：恢复修改前的布局。
# 左臂基座在原点，右臂基座位于 (0.35, -0.71, 0)，绕 Z 轴旋转 90°。
# 真机标定保持不变；仿真任务通过 task 配置提供对应的仿真标定。
_RIGHT_BASE_POS = (0.35, -0.71, 0.0)
_RIGHT_BASE_ORI = (0.0, 0.0, 1.5708)

# 相机安装位姿（Link7 -> cam_link_grasp），取自 mount_camera.py 的静态 TF。
# sim URDF 中 cam_link_grasp_joint 原点为 0（空 link），渲染/查询时需补上该偏移。
_CAM_LINK_NAMES = {"left": "cam_link_grasp", "right": "R_cam_link_grasp"}
_CAM_IN_LINK7_POS = (0.0708009744931787, 0.023445568410749785, 0.09466674449783057)
_CAM_IN_LINK7_QUAT = (0.010726, 0.004496, 0.729724, 0.683643)  # x,y,z,w


class SimServer:
    CAM_INTRINSICS = {
        "left": dict(fx=392.268, fy=392.268, cx=325.468, cy=242.282,
                     width=640, height=480, near=0.01, far=3.0),
        "right": dict(fx=392.268, fy=392.268, cx=325.468, cy=242.282,
                      width=640, height=480, near=0.01, far=3.0),
    }

    def __init__(self, vis=True, port=SIM_PORT, scene="warmcool"):
        self.scene = scene
        self.vis = vis
        self.side = "left"
        self._lock = threading.RLock()
        self.urdf_dir = os.path.join(
            os.path.dirname(__file__),
            "../smart_pick_and_place_ws/src/rm_description/urdf",
        )
        self.config = Config()
        self.CAM_INTRINSICS = {
            side: {
                **self.CAM_INTRINSICS.get(side, {}),
                **self.config.get_camera_intrinsics(side),
                "near": 0.01,
                "far": 3.0,
            }
            for side in ("left", "right")
        }
        self.robots = {}           # side -> ErdaijiRobot
        self.grippers = {}         # side -> gripper dict
        self.camera_link_ids = {}  # side -> link id or None
        # Simulation-only grasp state.  A real gripper closes through its
        # hardware driver; PyBullet needs an explicit constraint to model the
        # same object transport after a nearby contact.
        self.graspable_bodies = []
        self.graspable_initial_poses = {}
        self.placed_objects = set()
        self.held_objects = {"left": None, "right": None}
        self.suction_targets_left = {"left": None, "right": None}
        self.suction_diagnostics = {"left": {}, "right": {}}
        self.sim_log_dir = os.path.join(_THIS_DIR, "..", "..", "sim_log")
        os.makedirs(self.sim_log_dir, exist_ok=True)
        self.sim_capture_count = {"left": 0, "right": 0}
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
        self._load_scene(getattr(self, "scene", "warmcool"))
        self._load_arms()
        self._set_initial_home()
        self._disable_suction_object_collisions()
        self._index_cameras()
        self._init_grippers()

    def _load_arms(self):
        self.robots["left"] = self._make_robot(
            os.path.join(self.urdf_dir, "left_arm_bullet.urdf"),
            self.config.get_robot_model("left_gripper"),
            (0.0, 0.0, 0.0), (0, 0, 0),
        )
        self.robots["right"] = self._make_robot(
            os.path.join(self.urdf_dir, "right_arm.urdf"),
            self.config.get_robot_model("right_gripper"),
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

    def _set_initial_home(self):
        """Initialize both simulated arms at the project-level home pose.

        The URDF/gripper model has its own ``reset`` pose.  That pose is not
        guaranteed to match the named ``home`` pose used by the real-robot
        clients, so startup must explicitly apply robot_config.json's home
        joints and clear the model's pending reset command.
        """
        for side in ("left", "right"):
            pose = self.config.get_pose("home", side=side)
            if not isinstance(pose, dict):
                raise RuntimeError(f"missing {side} home pose in robot_config.json")
            joint_pose = deg2rad_list([pose[f"J{i}"] for i in range(1, 8)])
            struct = self.robots[side].robot_structs[f"{side}_arm"]
            struct.cmd_queue = []
            struct.count_execute_cmd = 0
            struct.reset_by_joint_states(joint_pose)
            struct.move_joint(joint_pose)
            struct.curr_js_pose = struct.get_joint_pose()
            struct.prev_js_pose = copy.deepcopy(struct.curr_js_pose)

    def _disable_suction_object_collisions(self):
        """Keep dynamic test objects from being knocked away before suction.

        This is a simulation-only contact model.  Twin still checks robot
        self-collision and floor clearance; the suction objects are attached
        by proximity when the gripper closes.
        """
        for robot in self.robots.values():
            link_ids = [-1] + list(range(p.getNumJoints(robot.id)))
            for body in self.graspable_bodies:
                for link_id in link_ids:
                    p.setCollisionFilterPair(
                        robot.id, body, link_id, -1, enableCollision=0
                    )

    def _spawn_shape(self, geom_type, mass=0.1, position=(0, 0, 0),
                     color=(1, 1, 1, 1), **kwargs):
        """Create a static/dynamic primitive body (box/sphere/cylinder)."""
        if geom_type == p.GEOM_CYLINDER:
            # 碰撞形状用 height，视觉形状用 length（pybullet API 差异）
            coll = p.createCollisionShape(geom_type, radius=kwargs["radius"],
                                          height=kwargs["height"])
            vis = p.createVisualShape(geom_type, radius=kwargs["radius"],
                                      length=kwargs["height"], rgbaColor=color)
        elif geom_type == p.GEOM_SPHERE:
            coll = p.createCollisionShape(geom_type, radius=kwargs["radius"])
            vis = p.createVisualShape(geom_type, radius=kwargs["radius"],
                                      rgbaColor=color)
        else:  # p.GEOM_BOX
            coll = p.createCollisionShape(geom_type, halfExtents=kwargs["halfExtents"])
            vis = p.createVisualShape(geom_type, halfExtents=kwargs["halfExtents"],
                                      rgbaColor=color)
        body = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=coll,
                                 baseVisualShapeIndex=vis, basePosition=position)
        p.changeDynamics(body, -1, lateralFriction=0.8,
                         rollingFriction=0.1, spinningFriction=0.1)
        return body

    def _spawn_apple(self, position, radius=0.04):
        """Red sphere + brown stem + green leaf, so YOLO-World recognises ``apple``.

        A bare flat-shaded sphere reads as ~0.07 confidence (below the 0.2
        detection threshold); adding a stem + leaf pushes it to ~0.44, above the
        plate/bowl false positives that otherwise hijack detection.
        """
        x, y, z = position
        self._spawn_shape(p.GEOM_SPHERE, mass=0.1, position=[x, y, z],
                          radius=radius, color=[0.78, 0.05, 0.05, 1])
        self._spawn_shape(p.GEOM_CYLINDER, mass=0.0,
                          position=[x, y, z + radius + 0.015],
                          radius=0.008, height=0.03, color=[0.40, 0.25, 0.10, 1])
        self._spawn_shape(p.GEOM_BOX, mass=0.0,
                          position=[x + 0.015, y, z + radius + 0.015],
                          halfExtents=[0.02, 0.006, 0.015],
                          color=[0.20, 0.60, 0.20, 1])

    def _load_scene(self, scene="warmcool"):
        if scene == "fruit":
            return self._load_scene_fruit()
        """Load one shared input platform and two classified outputs.

        Both arms take objects from ``in``.  The scheduler serializes access
        to that source surface, while the warm/cool output platforms remain
        separate resources.  All positions are in the unchanged historical
        dual-arm base frame.
        """
        # Platform side length is 0.16 m and thickness remains 0.05 m.
        # Keep the platform low, with its centre at z=0.05 m.  Its top
        # surface is z=0.075 m and the ball centres are generated above it.
        platform_half_extents = [0.08, 0.08, 0.025]
        in_platform_z = 0.05
        out1_platform_z = 0.05
        out2_platform_z = 0.05
        platform_poses = [
            (0.25, -0.35, in_platform_z),     # in: shared source
            (0.25, -0.15, out1_platform_z),   # out1: warm objects
            (-0.02, -0.60, out2_platform_z),  # out2: cool objects
        ]
        platform_colors = [
            [0.50, 0.50, 0.50, 1.0],  # in: neutral gray
            [0.95, 0.80, 0.10, 1.0],  # out1: warm/yellow
            [0.20, 0.45, 0.85, 1.0],  # out2: cool/blue
        ]
        for position, color in zip(platform_poses, platform_colors):
            self._spawn_shape(
                p.GEOM_BOX,
                mass=0.0,
                position=list(position),
                halfExtents=platform_half_extents,
                color=color,
            )

        # All objects are placed on the shared ``in`` platform.  Keep the
        # four centres inside the 0.16 m square with a small edge margin.
        ball_radius = 0.025
        in_platform_x, in_platform_y = 0.25, -0.35
        ball_z = in_platform_z + 0.025 + ball_radius
        ball_offsets = [
            (-0.045, -0.040), (-0.045, 0.040),
            (0.000, -0.040), (0.000, 0.040),
            (0.045, -0.040), (0.045, 0.040),
        ]
        warm_colors = [
            [0.95, 0.10, 0.10, 1.0],   # red
            [0.98, 0.70, 0.05, 1.0],   # yellow
            [0.95, 0.35, 0.70, 1.0],   # pink
            [0.95, 0.40, 0.05, 1.0],   # orange
        ]
        cool_colors = [
            [0.10, 0.35, 0.95, 1.0],   # blue
            [0.20, 0.75, 0.95, 1.0],   # cyan
            [0.45, 0.15, 0.85, 1.0],   # purple
            [0.10, 0.65, 0.55, 1.0],   # teal
        ]
        input_objects = [
            (ball_offsets[0], warm_colors[0]),
            (ball_offsets[1], warm_colors[1]),
            (ball_offsets[2], warm_colors[2]),
            (ball_offsets[3], cool_colors[0]),
            (ball_offsets[4], cool_colors[1]),
            (ball_offsets[5], cool_colors[2]),
        ]
        for (dx, dy), color in input_objects:
            body = self._spawn_shape(
                p.GEOM_SPHERE,
                mass=0.01,
                position=[in_platform_x + dx, in_platform_y + dy, ball_z],
                radius=ball_radius,
                color=color,
            )
            self.graspable_bodies.append(body)
            self.graspable_initial_poses[body] = (
                (float(in_platform_x + dx), float(in_platform_y + dy), float(ball_z)),
                (0.0, 0.0, 0.0, 1.0),
            )

    def _spawn_fruit(self, position, color, radius=0.04, stem_color=None):
        """Sphere + stem + leaf so YOLO-World recognises the fruit class.

        Same trick as ``_spawn_apple``: a bare flat-shaded sphere reads as a
        low-confidence blob, while the stem/leaf detail lifts it above the
        detection threshold.  Returns the main sphere body so callers can
        register it as graspable.
        """
        x, y, z = position
        body = self._spawn_shape(
            p.GEOM_SPHERE, mass=0.05, position=[x, y, z],
            radius=radius, color=color,
        )
        self._spawn_shape(
            p.GEOM_CYLINDER, mass=0.0,
            position=[x, y, z + radius + 0.012],
            radius=0.006, height=0.024,
            color=stem_color or [0.40, 0.25, 0.10, 1],
        )
        self._spawn_shape(
            p.GEOM_BOX, mass=0.0,
            position=[x + 0.014, y, z + radius + 0.012],
            halfExtents=[0.018, 0.005, 0.012],
            color=[0.20, 0.60, 0.20, 1],
        )
        return body

    def _load_scene_fruit(self):
        """Fruit sorting scene: apple + orange on the shared input platform,
        a white plate (out1 station) and a purple bowl (out2 station) as
        drop targets.  Objectives use generic ``apple``/``orange`` and
        ``plate``/``bowl`` prompts; no colour words (lighting-robust).
        """
        platform_half_extents = [0.08, 0.08, 0.025]
        base_z = 0.05
        poses = [
            (0.25, -0.35, base_z),    # shared input platform
            (0.25, -0.15, base_z),    # plate station (out1)
            (-0.02, -0.60, base_z),   # bowl station (out2)
        ]
        colors = [
            [0.50, 0.50, 0.50, 1.0],  # neutral gray input
            [0.92, 0.92, 0.92, 1.0],  # white plate
            [0.55, 0.20, 0.75, 1.0],  # purple bowl
        ]
        for position, color in zip(poses, colors):
            self._spawn_shape(
                p.GEOM_BOX, mass=0.0, position=list(position),
                halfExtents=platform_half_extents, color=color,
            )

        # Container look-alikes on top of the stations: a flat wide disc as
        # the plate and a taller disc as the bowl.  Solid discs stand in for
        # hollow tableware; placement targets are their top faces, which is
        # enough to exercise the grasp/place pipeline.
        plate_x, plate_y = 0.25, -0.15
        plate_top = base_z + 0.025
        self._spawn_shape(
            p.GEOM_CYLINDER, mass=0.0,
            position=[plate_x, plate_y, plate_top + 0.008],
            radius=0.095, height=0.016, color=[0.92, 0.92, 0.92, 1],
        )
        bowl_x, bowl_y = -0.02, -0.60
        bowl_top = base_z + 0.025
        self._spawn_shape(
            p.GEOM_CYLINDER, mass=0.0,
            position=[bowl_x, bowl_y, bowl_top + 0.018],
            radius=0.075, height=0.036, color=[0.55, 0.20, 0.75, 1],
        )

        # Two fruits on the shared input platform.
        in_x, in_y = 0.25, -0.35
        top = base_z + 0.025
        apple_body = self._spawn_fruit(
            [in_x - 0.045, in_y + 0.03, top + 0.04],
            color=[0.78, 0.05, 0.05, 1], radius=0.04,
        )
        self.graspable_bodies.append(apple_body)
        self.graspable_initial_poses[apple_body] = (
            (in_x - 0.045, in_y + 0.03, top + 0.04), (0.0, 0.0, 0.0, 1.0),
        )
        orange_body = self._spawn_fruit(
            [in_x + 0.045, in_y - 0.03, top + 0.037],
            color=[0.95, 0.45, 0.05, 1], radius=0.037,
        )
        self.graspable_bodies.append(orange_body)
        self.graspable_initial_poses[orange_body] = (
            (in_x + 0.045, in_y - 0.03, top + 0.037), (0.0, 0.0, 0.0, 1.0),
        )

    def _gripper_link_pose(self, side):
        robot = self.robots[side]
        # Attach to the visual gripper's fingertip/end-effector frame.  Link7
        # is the arm flange and can be several centimetres behind the palm;
        # using it made a large-radius suction model look like a false grasp.
        arm_cfg = self.config.get_arm_config(side)
        link_name = arm_cfg.get(
            "hand_effector_name",
            "L_gripper_endeffector" if side == "left" else "R_gripper_endeffector",
        )
        link_id = robot.linkName_to_id.get(link_name)
        if link_id is None:
            return None
        return p.getLinkState(robot.id, link_id, computeForwardKinematics=True)[:2]

    def _suction_config(self):
        cfg = self.config.shared.get("sim_suction", {})
        return {
            "enabled": bool(cfg.get("enabled", True)),
            "radius_m": float(cfg.get("radius_m", 0.085)),
        }

    def _release_suction(self, side):
        held = self.held_objects.get(side)
        if held is None:
            return
        constraint_id = held.get("constraint_id")
        if constraint_id is not None:
            try:
                p.removeConstraint(constraint_id)
            except Exception:
                pass
        body = held.get("body")
        robot = self.robots[side]
        if body is not None:
            p.setCollisionFilterPair(robot.id, body, -1, -1, enableCollision=1)
        self.held_objects[side] = None
        if body is not None:
            self.placed_objects.add(body)

    def _try_suction(self, side):
        """Attach the nearest simulated object within the suction radius.

        This is deliberately a proximity model, not a planner bypass: the
        arm must already have reached the Twin-generated grasp trajectory.
        Only dynamic scene objects in ``graspable_bodies`` are eligible.
        """
        cfg = self._suction_config()
        if not cfg["enabled"] or self.held_objects.get(side) is not None:
            return None
        link_pose = self._gripper_link_pose(side)
        if link_pose is None:
            return None
        link_pos, link_orn = link_pose
        candidates = []
        nearest_distance = None
        for body in self.graspable_bodies:
            if any(item and item.get("body") == body
                   for item in self.held_objects.values()):
                continue
            pos, orn = p.getBasePositionAndOrientation(body)
            distance = float(np.linalg.norm(np.asarray(pos) - np.asarray(link_pos)))
            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
            if distance <= cfg["radius_m"]:
                candidates.append((distance, body, pos, orn))
        if not candidates:
            self.suction_diagnostics[side] = {
                "object_detected": False,
                "nearest_distance_m": nearest_distance,
                "link_pos": list(link_pos),
            }
            return None
        target_left = self.suction_targets_left.get(side)
        if target_left is not None:
            target = np.asarray(target_left, dtype=float)
            _, body, object_pos, object_orn = min(
                candidates,
                key=lambda item: float(np.linalg.norm(
                    np.asarray(item[2], dtype=float) - target
                )),
            )
        else:
            _, body, object_pos, object_orn = min(candidates, key=lambda item: item[0])
        inv_pos, inv_orn = p.invertTransform(link_pos, link_orn)
        parent_pos, parent_orn = p.multiplyTransforms(
            inv_pos, inv_orn, object_pos, object_orn
        )
        robot = self.robots[side]
        link_name = self.config.get_arm_config(side).get(
            "hand_effector_name",
            "L_gripper_endeffector" if side == "left" else "R_gripper_endeffector",
        )
        link_id = robot.linkName_to_id[link_name]
        constraint_id = p.createConstraint(
            parentBodyUniqueId=robot.id,
            parentLinkIndex=link_id,
            childBodyUniqueId=body,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=parent_pos,
            childFramePosition=[0, 0, 0],
            parentFrameOrientation=parent_orn,
            childFrameOrientation=[0, 0, 0, 1],
        )
        p.setCollisionFilterPair(robot.id, body, -1, -1, enableCollision=0)
        self.held_objects[side] = {"body": body, "constraint_id": constraint_id}
        self.suction_targets_left[side] = None
        self.suction_diagnostics[side] = {
            "object_detected": True,
            "nearest_distance_m": float(np.linalg.norm(
                np.asarray(object_pos) - np.asarray(link_pos)
            )),
            "link_pos": list(link_pos),
            "body": body,
        }
        return {"body": body, "distance_m": float(np.linalg.norm(
            np.asarray(object_pos) - np.asarray(link_pos)
        ))}


    def _index_cameras(self):
        for side, name in _CAM_LINK_NAMES.items():
            robot = self.robots[side]
            if name in robot.linkName_to_id:
                self.camera_link_ids[side] = robot.linkName_to_id[name]
            else:
                # right_arm.urdf intentionally omits the R_cam_link_grasp
                # static link (the real TF is published by
                # mount_camera_right.py).  Fall back to the flange link:
                # _camera_world_pose composes the same Link7->camera
                # extrinsic (_CAM_IN_LINK7_*) for both arms, so the flange
                # link gives an identical camera pose.
                fallback = "R_Link7" if side == "right" else "Link7"
                self.camera_link_ids[side] = robot.linkName_to_id.get(fallback)

    def _camera_world_pose(self, side):
        """Return the simulated RGB-D camera pose in world coordinates.

        ``scene_fixed`` is an explicit simulation sensor mode. It provides a
        repeatable view of the whole task scene; the planner still obtains
        object locations only from RGB-D pixels, never from simulator state.
        """
        sim_camera = self.config.shared.get("sim_camera", {})
        if sim_camera.get("mode") == "scene_fixed":
            position = np.asarray(
                sim_camera.get("position", [0.25, -0.95, 0.85]),
                dtype=float,
            )
            target = np.asarray(
                sim_camera.get("target", [0.18, -0.38, 0.30]),
                dtype=float,
            )
            forward = target - position
            forward /= max(np.linalg.norm(forward), 1e-9)
            reference_up = np.asarray([0.0, 0.0, 1.0], dtype=float)
            right = np.cross(forward, reference_up)
            right /= max(np.linalg.norm(right), 1e-9)
            image_up = np.cross(right, forward)
            image_up /= max(np.linalg.norm(image_up), 1e-9)
            # Renderer uses local +Z as optical forward and local -Y as
            # image-up, matching the RGB-D projection convention below.
            camera_to_world = np.column_stack((right, -image_up, forward))
            orn = Rotation.from_matrix(camera_to_world).as_quat()
            return position.tolist(), orn.tolist()

        """相机光心世界位姿 = Link7 位姿 复合真实安装偏移（mount_camera.py）。"""
        robot = self.robots[side]
        pos7, orn7 = p.getLinkState(robot.id, self.camera_link_ids[side])[:2]
        return p.multiplyTransforms(pos7, orn7,
                                    _CAM_IN_LINK7_POS, _CAM_IN_LINK7_QUAT)

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
            "open_angle": 0.72,  # rad；≤ L_finger_joint 上限 0.725，避免顶限震荡
        }

    def _step(self, n=1):
        with self._lock:
            for _ in range(n):
                for robot in self.robots.values():
                    robot.apply_actions()
                p.stepSimulation()
                held_bodies = {
                    item.get("body") for item in self.held_objects.values()
                    if item is not None
                }
                for body in self.graspable_bodies:
                    if body in held_bodies or body in self.placed_objects:
                        continue
                    initial = self.graspable_initial_poses.get(body)
                    if initial is None:
                        continue
                    p.resetBasePositionAndOrientation(body, *initial)
                    p.resetBaseVelocity(body, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
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
        if cmd == "set_suction_target":
            side = req.get("side", "left")
            point = req.get("point_left")
            if isinstance(point, list) and len(point) == 3:
                self.suction_targets_left[side] = [float(v) for v in point]
                return {"value": True, "info": {}}
            return {"value": False, "info": {"error": "invalid suction target"}}
        if cmd == "get_sim_objects":
            with self._lock:
                held = {
                    side: None if item is None else item.get("body")
                    for side, item in self.held_objects.items()
                }
                objects = []
                for body in self.graspable_bodies:
                    pos, _ = p.getBasePositionAndOrientation(body)
                    objects.append({
                        "body": body,
                        "pos": list(pos),
                        "held": body in [value for value in held.values() if value is not None],
                        "placed": body in self.placed_objects,
                    })
            return {"value": True, "info": {"objects": objects, "held": held}}
        if cmd == "get_rgbd":
            return self._get_rgbd(req)
        if cmd == "get_link_pose":
            return self._get_link_pose(req)
        if cmd == "get_base_pose":
            return self._get_base_pose(req)
        return {"value": False, "info": {"error": f"unknown cmd {cmd}"}}

    # -- handlers ------------------------------------------------------
    def _arm_struct(self, side):
        return self.robots[side].robot_structs[f"{side}_arm"]

    def _get_joint_state(self, req):
        side = req.get("side", "left")
        with self._lock:
            js_rad = self._arm_struct(side).get_joint_pose()  # radians, 7-list
        return {"value": True, "info": {"js_deg": rad2deg_list(js_rad)}}

    def _move_joints_smooth(self, side, target_js_rad):
        """Drive the arm to `target_js_rad` (radians) at bounded velocity.

        The old code teleported via ``reset_by_joint_states`` (``p.resetJointState``)
        and stepped a fixed 3-6 times, so the arm snapped to each target. Position
        control with a capped ``maxVelocity`` + enough steps makes it sweep smoothly.
        """
        arm = self._arm_struct(side)
        start = np.asarray(arm.get_joint_pose(), dtype=float)
        target = np.asarray(target_js_rad, dtype=float)
        max_delta = float(np.max(np.abs(target - start)))
        arm.maxvel = ARM_MAX_VEL
        arm.move_joint(target)
        n_steps = max(int(max_delta / (ARM_MAX_VEL * SIM_STEP_DELAY)) + 1, 4)
        for _ in range(n_steps):
            self._step(1)

    def _execute_trajectory(self, req):
        side = req.get("side", "left")
        trajectory = req.get("trajectory", [])
        if not trajectory:
            return {"value": False, "info": {"error": "empty trajectory"}}
        with self._lock:
            for wp in trajectory:
                self._move_joints_smooth(side, deg2rad_list(list(wp)))
        return {"value": True, "info": {"n_waypoints": len(trajectory)}}

    def _move_to_pose(self, req):
        side = req.get("side", "left")
        pose = req.get("pose", {})
        js_deg = [pose.get(f"J{i}", 0.0) for i in range(1, 8)]
        with self._lock:
            self._move_joints_smooth(side, deg2rad_list(js_deg))
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
                                    p.POSITION_CONTROL, angle,
                                    force=200.0, maxVelocity=1.0)
            for name, cid in gripper["child_ids"].items():
                mult = gripper["children"][name]
                p.setJointMotorControl2(rid, cid, p.POSITION_CONTROL,
                                        angle * mult,
                                        force=200.0, maxVelocity=1.0)
            self._step(6)
            if action == "close" or value <= 0:
                attached = self._try_suction(side)
            else:
                attached = None
                self._release_suction(side)
        return {
            "value": True,
            "info": {
                "angle": angle,
                "value": value,
                "suction_enabled": self._suction_config()["enabled"],
                "object_detected": attached is not None if action == "close" or value <= 0 else False,
                "suction_distance_m": None if attached is None else attached["distance_m"],
                "nearest_distance_m": self.suction_diagnostics.get(side, {}).get(
                    "nearest_distance_m"
                ),
                "suction_link_pos": self.suction_diagnostics.get(side, {}).get(
                    "link_pos"
                ),
            },
        }

    def _get_rgbd(self, req):
        side = req.get("side", self.side)
        camera_link_id = self.camera_link_ids.get(side)
        if camera_link_id is None:
            return {"value": False, "info": {"error": f"no camera link for {side}"}}
        intr = self.CAM_INTRINSICS[side]
        robot = self.robots[side]
        with self._lock:
            pos, orn = self._camera_world_pose(side)
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
        self._save_sim_capture(side, rgb_arr, depth_mm)
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

    def _save_sim_capture(self, side, rgb_arr, depth_mm):
        """Persist every simulated RGB-D capture without blocking the reply on errors."""
        try:
            from PIL import Image

            with self._lock:
                self.sim_capture_count[side] += 1
                index = self.sim_capture_count[side]
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            prefix = os.path.join(
                self.sim_log_dir, f"{stamp}_{side}_{index:04d}"
            )
            Image.fromarray(rgb_arr).save(f"{prefix}_rgb.png")
            Image.fromarray(np.asarray(depth_mm, dtype=np.uint16)).save(
                f"{prefix}_depth.png"
            )
        except Exception as exc:  # noqa: BLE001
            # Camera logging is diagnostic; a disk error must not make the
            # simulated camera service unavailable.
            cprint(f"[SimServer] capture log failed: {exc}", "yellow")

    def _get_link_pose(self, req):
        side = req.get("side", "left")
        name = req.get("link")
        robot = self.robots[side]
        if name not in robot.linkName_to_id:
            return {"value": False, "info": {"error": f"unknown link {name}"}}
        with self._lock:
            if name == _CAM_LINK_NAMES.get(side):
                pos, orn = self._camera_world_pose(side)
            else:
                pos, orn = p.getLinkState(robot.id, robot.linkName_to_id[name])[:2]
        return {"value": True, "info": {"pos": list(pos), "orn": list(orn)}}

    def _get_base_pose(self, req):
        side = req.get("side", "left")
        robot = self.robots[side]
        with self._lock:
            pos, orn = p.getBasePositionAndOrientation(robot.id)
        return {"value": True, "info": {"pos": list(pos), "orn": list(orn)}}


if __name__ == "__main__":
    argv = rospy.myargv()
    parser = argparse.ArgumentParser()
    parser.add_argument("--novis", action="store_true")
    parser.add_argument("--port", type=int, default=SIM_PORT)
    parser.add_argument("--scene", default="warmcool",
                        choices=["warmcool", "fruit"])
    args, _ = parser.parse_known_args(argv[1:])
    rospy.init_node("sim_server", anonymous=True)
    server = SimServer(vis=not args.novis, port=args.port, scene=args.scene)
    while not rospy.is_shutdown():
        server._step()
