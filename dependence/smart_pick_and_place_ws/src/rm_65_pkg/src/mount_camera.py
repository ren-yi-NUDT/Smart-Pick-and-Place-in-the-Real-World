#!/home/zz/anaconda3/envs/anygrasp/bin/python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations as tr
import numpy as np
import json
from pathlib import Path


def load_calibrated_extrinsic():
    """Load the calibrated Link7 -> camera transform from the project config."""
    config_path = Path(__file__).resolve().parents[5] / "robot_config.json"
    with config_path.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    extrinsic = config["arms"]["left"]["camera_extrinsic"]
    if extrinsic["parent_frame"] != "Link7" or extrinsic["child_frame"] != "cam_link_grasp":
        raise RuntimeError("Left camera extrinsic frame names do not match the TF frames")
    matrix = np.asarray(extrinsic["matrix"], dtype=float)
    if matrix.shape != (4, 4):
        raise RuntimeError(f"Expected a 4x4 left camera extrinsic, got {matrix.shape}")
    return matrix

class MountCamera():
    def __init__(self, name, parent_frame, child_frame):
        super().__init__()
        node_name = 'static_tf2_broadcaster_' + name
        rospy.init_node(f'{node_name}', anonymous=True)
        self.parent_frame = parent_frame
        self.child_frame = child_frame

    def matrix_to_list(self, matrix_4x4):
        translation = tr.translation_from_matrix(matrix_4x4)
        quaternion = tr.quaternion_from_matrix(matrix_4x4)
        return list(translation) + list(quaternion)

    def calculate_and_broadcast(self):
        T_matrix = load_calibrated_extrinsic()
        L = self.matrix_to_list(T_matrix)
        # N = [-0.059, -0.000, 0.000,
        #      0.501, -0.498, 0.501, 0.501]
        # N = [0, 0, 0,
        #      0, 0, 0, 1]
        # L = self.calculate_transform(M, N)
        self.publish_static_transform(L, parent_frame, child_frame)
        rospy.spin()

    def calculate_transform(self, M, N):
        translation_M = M[:3]
        quaternion_M = M[3:]
        matrix_M = tr.concatenate_matrices(
            tr.translation_matrix(translation_M),
            tr.quaternion_matrix(quaternion_M)
        )
        translation_N = N[:3]
        quaternion_N = N[3:]
        matrix_N = tr.concatenate_matrices(
            tr.translation_matrix(translation_N),
            tr.quaternion_matrix(quaternion_N)
        )
        matrix_N_inv = tr.inverse_matrix(matrix_N)
        matrix_L = np.dot(matrix_M, matrix_N_inv)
        translation_L = tr.translation_from_matrix(matrix_L)
        quaternion_L = tr.quaternion_from_matrix(matrix_L)

        return list(translation_L) + list(quaternion_L)

    def publish_static_transform(self, L, parent_frame, child_frame):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent_frame
        static_transformStamped.child_frame_id = child_frame
        static_transformStamped.transform.translation.x = L[0]
        static_transformStamped.transform.translation.y = L[1]
        static_transformStamped.transform.translation.z = L[2]
        static_transformStamped.transform.rotation.x = L[3]
        static_transformStamped.transform.rotation.y = L[4]
        static_transformStamped.transform.rotation.z = L[5]
        static_transformStamped.transform.rotation.w = L[6]

        broadcaster.sendTransform(static_transformStamped)
        rospy.loginfo("Published static transform from {} to {}".format(parent_frame, child_frame))

if __name__ == '__main__':
    name = 'realsense'
    parent_frame = 'Link7'
    child_frame = 'cam_link_grasp'
    mc = MountCamera(name, parent_frame, child_frame)
    mc.calculate_and_broadcast()
