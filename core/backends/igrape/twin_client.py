"""
Igrape-bot3 twin client adapter.

Wraps the SmartGraspPlanner (PyBullet IK) to provide the same interface
as TwinClient (trajectory generation service).
"""

import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint

from core.abc import BaseTwinClient
from core.backends.igrape._joint_map import (
    J_NAME_TO_MOTOR_ID, motor_traj_to_j_rad,
)


class IgrapeTwinClient(BaseTwinClient):
    """Twin client adapter using local PyBullet IK."""

    def __init__(self, **kwargs):
        self._connected = False
        self._planner = None
        self._cam_in_base = None

    def connect(self) -> bool:
        # Ensure Igrape workspace is on sys.path
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.abspath(__file__))))),
            "igrape_config.json"
        )
        with open(config_path) as f:
            cfg = json.load(f)
        igrape_root = cfg["igrape_root"]
        if igrape_root not in sys.path:
            sys.path.insert(0, igrape_root)

        self._cam_in_base = np.eye(4)[None, ...]
        self._connected = True
        cprint("[IgrapeTwin] Connected (PyBullet IK, lazy init)", "green")
        return True

    def close(self) -> None:
        self._connected = False

    def _ensure_planner(self):
        if self._planner is not None:
            return
        from body.utils_body.smart_grasp_planner import SmartGraspPlanner
        self._planner = SmartGraspPlanner(self._cam_in_base)

    def set_cam_in_base(self, cam_in_base):
        """Update camera-to-base transform (called by transforms adapter)."""
        self._cam_in_base = cam_in_base
        if self._planner is not None:
            self._planner.cam_in_base = cam_in_base

    def call_service(self, srv_type: str, cnfg: dict) -> dict:
        if srv_type in ("trajectory_generation", "trajectory_generation2"):
            return self.generate_trajectory2(cnfg)
        if srv_type in ("reachability_check", "collision_check", "IK_calculation"):
            return self._ik_check(cnfg)
        raise ValueError(f"Unknown twin service type: {srv_type}")

    def generate_trajectory2(self, cnfg: dict) -> dict:
        """Generate a joint-space trajectory for the given target EE poses.

        cnfg format:
        {
            "target_pose": [[x, y, z, qx, qy, qz, qw], ...],
            "current_js": [rad, rad, ...],  # 7 values in radians (J1-J7 order)
            "struct": "left_arm"
        }

        Returns:
        {
            "value": True/False,
            "info": {
                "trajectory": [[rad_j1, ..., rad_j7], ...],
                "trajectory_ee": [...],
                "infos": {"is_collided": False}
            }
        }
        """
        self._ensure_planner()

        target_poses = cnfg.get("target_pose", [])
        current_js = cnfg.get("current_js", [])

        if not target_poses:
            return {"value": False, "info": {}}

        # Set current joint state in PyBullet
        if current_js:
            self._set_pybullet_joints(current_js)

        # For each target pose, compute IK to get joint angles
        waypoints = []
        for pose in target_poses:
            target_pos = np.array(pose[:3])
            target_quat = np.array(pose[3:7])  # [qx, qy, qz, qw]

            # Build 4x4 target matrix
            target_mat = np.eye(4)
            target_mat[:3, :3] = R.from_quat(target_quat).as_matrix()
            target_mat[:3, 3] = target_pos

            # Solve IK
            joint_angles = self._solve_ik(target_mat)
            if joint_angles is None:
                cprint("[IgrapeTwin] IK failed for target pose", "red")
                return {"value": False, "info": {}}

            waypoints.append(joint_angles)

        # Interpolate between waypoints for smooth trajectory
        if len(waypoints) == 1:
            trajectory_rad = motor_traj_to_j_rad([waypoints[0]])
            return {
                "value": True,
                "info": {
                    "trajectory": trajectory_rad,
                    "trajectory_ee": [],
                    "infos": {"is_collided": False},
                },
            }

        # Multiple waypoints: simple linear interpolation in joint space
        full_traj = self._interpolate_waypoints(waypoints, steps_per_segment=20)
        trajectory_rad = motor_traj_to_j_rad(full_traj)

        return {
            "value": True,
            "info": {
                "trajectory": trajectory_rad,
                "trajectory_ee": [],
                "infos": {"is_collided": False},
            },
        }

    def _set_pybullet_joints(self, js_rad):
        """Set PyBullet robot joint states from J1-J7 radian list."""
        import pybullet as p

        planner_names = self._planner.movable_joint_names
        planner_indices = self._planner.movable_joint_indices

        # Motor ID -> PyBullet joint name mapping from SmartGraspPlanner
        from body.utils_body.smart_grasp_planner import JOINT_NAME_TO_REAL_ID, REAL_ID_TO_JOINT_NAME

        for i, j_name in enumerate(["J1", "J2", "J3", "J4", "J5", "J6", "J7"]):
            motor_id = J_NAME_TO_MOTOR_ID[j_name]
            # Find the PyBullet joint name for this motor ID
            pb_names = [k for k, v in JOINT_NAME_TO_REAL_ID.items() if v == motor_id]
            if not pb_names:
                continue
            pb_name = pb_names[0]
            if pb_name in planner_names:
                idx = planner_indices[planner_names.index(pb_name)]
                p.resetJointState(self._planner.robot_id, idx, js_rad[i])

    def _solve_ik(self, target_4x4: np.ndarray) -> dict | None:
        """Solve IK for a target EE pose using PyBullet.

        Returns: {motor_id: angle_rad, ...} or None if failed.
        """
        import pybullet as p
        from body.utils_body.smart_grasp_planner import JOINT_NAME_TO_REAL_ID

        target_pos = target_4x4[:3, 3]
        target_orn = R.from_matrix(target_4x4[:3, :3]).as_quat()  # [x, y, z, w]

        ee_index = 22  # SmartGraspPlanner's EE_LINK_INDEX
        joint_positions = p.calculateInverseKinematics(
            self._planner.robot_id,
            ee_index,
            target_pos,
            target_orn,
            lowerLimits=self._planner.ll,
            upperLimits=self._planner.ul,
            jointRanges=self._planner.jr,
            restPoses=self._planner.rp,
            maxNumIterations=100,
            residualThreshold=0.01,
        )

        if not joint_positions:
            return None

        # Map PyBullet joint indices to motor IDs using planner's own mapping
        result = {}
        for i, joint_name in enumerate(self._planner.movable_joint_names):
            if joint_name in JOINT_NAME_TO_REAL_ID:
                motor_id = JOINT_NAME_TO_REAL_ID[joint_name]
                result[motor_id] = joint_positions[i]

        return result if len(result) == 7 else None

    def _interpolate_waypoints(self, waypoints: list, steps_per_segment: int = 20) -> list:
        """Linear interpolation between waypoints in joint space."""
        full_traj = []
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            motor_ids = sorted(set(start.keys()) | set(end.keys()))
            for step in range(steps_per_segment):
                alpha = step / steps_per_segment
                wp = {}
                for mid in motor_ids:
                    s = start.get(mid, 0.0)
                    e = end.get(mid, 0.0)
                    wp[mid] = s + alpha * (e - s)
                full_traj.append(wp)
        full_traj.append(waypoints[-1])
        return full_traj

    def _ik_check(self, cnfg: dict) -> dict:
        """Reachability / collision check (simplified)."""
        result = self.generate_trajectory2(cnfg)
        return {
            "value": result["value"],
            "info": {
                "is_reached": result["value"],
                "delta_xyz": 0.0,
                "delta_rpy": [0.0, 0.0, 0.0],
                "is_collided": False,
            },
        }
