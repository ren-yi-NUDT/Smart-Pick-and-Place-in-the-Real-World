"""
Igrape-bot3 transforms adapter.

Uses ROS2 tf2 for coordinate frame lookups, replacing the ROS1 tf wrapper.
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from core.abc import BaseTransforms


class IgrapeTransforms(BaseTransforms):
    """Transform adapter using ROS2 tf2."""

    def __init__(self):
        from core.backends.igrape._ros2_context import ROS2Context
        ctx = ROS2Context.get()
        self._node = ctx.node

        from tf2_ros import Buffer, TransformListener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

    def get_transform_from_frame_to_frame(self, from_frame: str, to_frame: str):
        """Look up the homogeneous transform between two frames.

        Returns: (4x4_matrix, euler_list, quat_list)
        """
        import rclpy.time

        retries = 5
        backoff = 0.5

        for i in range(retries):
            try:
                transform = self._tf_buffer.lookup_transform(
                    from_frame, to_frame, rclpy.time.Time()
                )
                t = transform.transform.translation
                r = transform.transform.rotation

                position = [t.x, t.y, t.z]
                quaternion = [r.x, r.y, r.z, r.w]

                translation_matrix = np.eye(4)
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

            except Exception as e:
                if i < retries - 1:
                    time.sleep(backoff * (i + 1))
                else:
                    raise RuntimeError(
                        f"Failed to look up transform {from_frame} -> {to_frame} "
                        f"after {retries} retries: {e}"
                    )
