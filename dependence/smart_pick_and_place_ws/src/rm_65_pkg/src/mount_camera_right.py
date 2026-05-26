#!/home/zz/anaconda3/envs/anygrasp/bin/python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations as tr
import numpy as np
import os, sys

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
        # 右臂相机外参：R_Link7 → R_cam_link_grasp
        # 与左臂使用相同的法兰安装，因此变换矩阵一致
        T_matrix = np.array([
            [-0.06503404712968464, -0.997644867154117, 0.021801187008471494, 0.0708009744931787],
            [0.9978377577069856, -0.0652237167782832, -0.00810407699381243, 0.023445568410749785],
            [0.00950694526276967, 0.021227006634725668, 0.9997294794998796, 0.09466674449783057],
            [0.0, 0.0, 0.0, 1.0]
        ])
        L = self.matrix_to_list(T_matrix)
        self.publish_static_transform(L, self.parent_frame, self.child_frame)
        rospy.spin()

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
    name = 'realsense_right'
    parent_frame = 'R_Link7'
    child_frame = 'R_cam_link_grasp'
    mc = MountCamera(name, parent_frame, child_frame)
    mc.calculate_and_broadcast()
