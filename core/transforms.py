"""
Coordinate transforms and geometric utilities.

Merges ``transformation.py`` (TransformationUtil) and ``utils.py``
(all standalone functions) into a single module.

Usage:
    from core.transforms import TransformationUtil
    from core.transforms import (
        graspcam2pixel, pixel_to_camera_point, pixel_to_camera_point2,
        self_rotation_np, rpy_to_vector, transform_world_to_camera,
        self_rotation_inv, visualization,
    )
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation as R

# Optional heavy imports -- only needed when running with a ROS / Open3D
# environment.  The functions that depend on them will fail gracefully.

try:
    import rospy
    import tf
    import tf2_ros
except ImportError:
    rospy = None
    tf = None
    tf2_ros = None

try:
    import open3d as o3d
    from graspnetAPI import GraspGroup
except ImportError:
    o3d = None
    GraspGroup = None


# ======================================================================
# TransformationUtil  (from transformation.py)
# ======================================================================

class TransformationUtil:
    """ROS-TF wrapper for looking up transforms between frames."""

    def __init__(self):
        if rospy is None:
            raise ImportError("rospy is required for TransformationUtil")
        rospy.init_node("transformation_util")

    def get_transform_from_frame_to_frame(self, from_frame: str, to_frame: str):
        """Look up the homogeneous transform from *from_frame* to *to_frame*.

        Returns
        -------
        translation_matrix : np.ndarray (4, 4)
        transformation_euler : list  [tx, ty, tz, rx, ry, rz]  (radians)
        transformation_quat : list  [tx, ty, tz, qx, qy, qz, qw]
        """
        listener = tf.TransformListener()
        listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))

        retries = 5
        backoff_factor = 0.5

        for i in range(retries):
            try:
                (position, quaternion) = listener.lookupTransform(
                    from_frame, to_frame, rospy.Time(0)
                )
                translation_matrix = np.eye(4, 4)
                orientation = R.from_quat(quaternion).as_matrix()
                euler = R.from_quat(quaternion).as_euler("xyz", degrees=False)
                translation_matrix[:3, 3] = position
                translation_matrix[:3, :3] = orientation
                transformation_euler = [
                    position[0], position[1], position[2],
                    euler[0], euler[1], euler[2],
                ]
                transformation_quat = position + quaternion
                return translation_matrix, transformation_euler, transformation_quat
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                time.sleep(backoff_factor * (i + 1))

        raise RuntimeError(
            f"Failed to look up transform {from_frame} -> {to_frame} after {retries} retries"
        )


# ======================================================================
# Utility functions  (from utils.py)
# ======================================================================

# -- camera intrinsics --
_RS_RIGHT_FX = 385.9778137207031
_RS_RIGHT_FY = 385.34674072265625
_RS_RIGHT_CX = 318.2220153808594
_RS_RIGHT_CY = 238.8162841796875

_RS_LEFT_FX = 392.26812744140625
_RS_LEFT_FY = 392.26812744140625
_RS_LEFT_CX = 325.4682312011719
_RS_LEFT_CY = 242.28213500976562


def _get_intrinsics(cam_type: str):
    if cam_type == "right":
        return _RS_RIGHT_FX, _RS_RIGHT_FY, _RS_RIGHT_CX, _RS_RIGHT_CY
    else:
        return _RS_LEFT_FX, _RS_LEFT_FY, _RS_LEFT_CX, _RS_LEFT_CY


def graspcam2pixel(grasping_pose, cam_type: str = "right"):
    """Project 3-D grasp translations to 2-D pixel coordinates.

    Parameters
    ----------
    grasping_pose : list[dict]
        Each dict must contain ``"trans"`` (3-element list).
    cam_type : ``"right"`` or ``"left"``

    Returns
    -------
    points_screen : np.ndarray (N, 2)
    grasping_pose : the original list (unchanged)
    """
    grasp_points = []
    for grasp in grasping_pose:
        translation = np.array(grasp["trans"])
        grasp_point = translation.reshape((-1, 3))
        grasp_points.append(grasp_point)

    grasp_points = np.concatenate(grasp_points, axis=0)

    X_c, Y_c, Z_c = grasp_points[:, 0], grasp_points[:, 1], grasp_points[:, 2]
    fx, fy, cx, cy = _get_intrinsics(cam_type)
    u = (fx * X_c / Z_c) + cx
    v = (fy * Y_c / Z_c) + cy
    points_screen = np.vstack((u, v)).T

    return points_screen[:, :2], grasping_pose


def pixel_to_camera_point(pixel_points, depth_image, cam_type: str = "right"):
    """Back-project pixel coordinates to 3-D camera-frame points using a
    depth image.

    Returns
    -------
    grasp_points_3d_m : np.ndarray (N, 3)  -- units: metres
    """
    fx, fy, cx, cy = _get_intrinsics(cam_type)
    u_points = pixel_points[:, 0].astype(np.int16)
    v_points = pixel_points[:, 1].astype(np.int16)
    depth_values = depth_image[v_points, u_points]
    grasp_points_cam = []
    for u, v, depth_value in zip(pixel_points[:, 0], pixel_points[:, 1], depth_values):
        Z_c = depth_value
        X_c = (u - cx) * Z_c / fx
        Y_c = (v - cy) * Z_c / fy
        grasp_points_cam.append([X_c, Y_c, Z_c])
    grasp_points_3d_m = np.stack(grasp_points_cam, axis=0) * 1e-3
    return grasp_points_3d_m


def pixel_to_camera_point2(pixel_points, depth_value_m, cam_type: str = "right"):
    """Back-project pixel coordinates to 3-D camera-frame points using a
    scalar / array depth value (metres).

    Returns
    -------
    grasp_points_3d_m : np.ndarray (N, 3)
    """
    fx, fy, cx, cy = _get_intrinsics(cam_type)

    u_points = pixel_points[:, 0]
    v_points = pixel_points[:, 1]
    if isinstance(depth_value_m, (int, float)):
        Z_c = np.full_like(u_points, depth_value_m, dtype=np.float32)
    else:
        Z_c = depth_value_m
    X_c = (u_points - cx) * Z_c / fx
    Y_c = (v_points - cy) * Z_c / fy
    grasp_points_3d_m = np.stack([X_c, Y_c, Z_c], axis=1)
    return grasp_points_3d_m


def self_rotation_np(pose: np.ndarray) -> np.ndarray:
    """Apply a fixed self-rotation to a 4x4 pose matrix."""
    transformation_matrix = np.array([
        [0, 0, 1, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ])
    return transformation_matrix @ pose


def rpy_to_vector(r, p, y, axis=None):
    """Convert roll, pitch, yaw (radians) to a unit direction vector.

    *axis* defaults to ``[1, 0, 0]`` when ``None``.
    """
    if axis is None:
        axis = [1, 0, 0]
    Rz = np.array([
        [np.cos(y), -np.sin(y), 0],
        [np.sin(y),  np.cos(y), 0],
        [0,          0,          1],
    ])
    Ry = np.array([
        [ np.cos(p), 0, np.sin(p)],
        [ 0,         1, 0         ],
        [-np.sin(p), 0, np.cos(p)],
    ])
    Rx = np.array([
        [1,  0,          0         ],
        [0,  np.cos(r), -np.sin(r)],
        [0,  np.sin(r),  np.cos(r)],
    ])
    rotation_matrix = Rz @ Ry @ Rx
    direction_vector = rotation_matrix @ np.array(axis)
    return direction_vector


def transform_world_to_camera(pose_matrix, T_base_to_cam):
    """Transform a pose from the world (base-link) frame to the camera frame."""
    T_base_to_cam_inv = np.linalg.inv(T_base_to_cam)
    transformed_pose = T_base_to_cam_inv @ pose_matrix
    return transformed_pose


def self_rotation_inv(pose=None):
    """Return the inverse rotation of the self-rotation convention used in
    ``self_rotation_np``.

    Returns a 3x3 rotation matrix.
    """
    transformation_matrix = np.array([
        [ 0,  0,  1,  0],
        [ 0,  1,  0,  0],
        [-1,  0,  0,  0],
        [ 0,  0,  0,  1],
    ])
    inverse_matrix = np.linalg.inv(transformation_matrix)
    return inverse_matrix[:3, :3]


def visualization(cloud, grasp_pose):
    """Visualize a point cloud and a single grasp gripper using Open3D."""
    gg = np.array([[
        0.17656013369560242,
        0.0575287826359272,
        0.029999999329447746,
        0.029999999329447746,
        grasp_pose[0][0], grasp_pose[0][1], grasp_pose[0][2],
        grasp_pose[1][0], grasp_pose[1][1], grasp_pose[1][2],
        grasp_pose[2][0], grasp_pose[2][1], grasp_pose[2][2],
        grasp_pose[0][3], grasp_pose[1][3], grasp_pose[2][3],
        -1,
    ]], dtype=np.float64)
    trans_mat = np.array([
        [1, 0,  0, 0],
        [0, 1,  0, 0],
        [0, 0, -1, 0],
        [0, 0,  0, 1],
    ])
    grippers = GraspGroup(gg).to_open3d_geometry_list()
    cloud = cloud.transform(trans_mat)
    gripper_pose = grippers[0].transform(trans_mat)
    gripper_pose.paint_uniform_color([0, 0, 1])
    o3d.visualization.draw_geometries([grippers[0], cloud])
