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
