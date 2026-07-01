#!/usr/bin/env python3
"""Merge /joint_states (left arm) and /right_joint_states (right arm) into one topic.

Publishes to /dual_joint_states so a single robot_state_publisher can drive
the combined dual-arm URDF.
"""
import rospy
from sensor_msgs.msg import JointState


class JointStateRelay:
    def __init__(self):
        self.left_msg = None
        self.right_msg = None
        self.pub = rospy.Publisher("/dual_joint_states", JointState, queue_size=10)

        rospy.Subscriber("/joint_states", JointState, self._left_cb)
        rospy.Subscriber("/right_joint_states", JointState, self._right_cb)

        rate = rospy.get_param("~rate", 30.0)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self._publish)

    def _left_cb(self, msg):
        self.left_msg = msg

    def _right_cb(self, msg):
        self.right_msg = msg

    def _publish(self, _event):
        out = JointState()
        out.header.stamp = rospy.Time.now()
        out.name = []
        out.position = []
        out.velocity = []
        out.effort = []

        if self.left_msg is not None:
            out.name.extend(self.left_msg.name)
            out.position.extend(self.left_msg.position)
            if self.left_msg.velocity:
                out.velocity.extend(self.left_msg.velocity)
            if self.left_msg.effort:
                out.effort.extend(self.left_msg.effort)

        if self.right_msg is not None:
            out.name.extend(self.right_msg.name)
            out.position.extend(self.right_msg.position)
            if self.right_msg.velocity:
                out.velocity.extend(self.right_msg.velocity)
            if self.right_msg.effort:
                out.effort.extend(self.right_msg.effort)

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("joint_state_relay")
    JointStateRelay()
    rospy.spin()
