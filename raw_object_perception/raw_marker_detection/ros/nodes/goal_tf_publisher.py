#!/usr/bin/env python

PACKAGE = 'raw_marker_detection'
NODE = 'goal_tf_publisher'
SERVICE = 'publish_goal'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

import numpy as np
import math
from tf.msg import tfMessage
from tf import TransformListener
import tf.transformations as tfs

from geometry_msgs.msg import TransformStamped
from raw_srvs.srv import PublishGoal


class TfFilter:
    def __init__(self, buffer_size):
        self.tf = TransformListener(True, rospy.Duration(5))
        self.target = ''
        self.buffer = np.zeros((buffer_size, 1))
        self.buffer_ptr = 0
        self.buffer_size = buffer_size
        self.tf_sub = rospy.Subscriber('tf', tfMessage, self.tf_cb)
        self.tf_pub = rospy.Publisher('tf', tfMessage)
        self.srv = rospy.Service(SERVICE, PublishGoal, self.publish_goal_cb)

    def tf_cb(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == self.target:
                time = self.tf.getLatestCommonTime(self.source, self.target)
                p, q = self.tf.lookupTransform(self.source, self.target, time)
                rm = tfs.quaternion_matrix(q)
                # assemble a new coordinate frame that has z-axis aligned with
                # the vertical direction and x-axis facing the z-axis of the
                # original frame
                z = rm[:3, 2]
                z[2] = 0
                axis_x = tfs.unit_vector(z)
                axis_z = tfs.unit_vector([0, 0, 1])
                axis_y = np.cross(axis_x, axis_z)
                rm = np.vstack((axis_x, axis_y, axis_z)).transpose()
                # shift the pose along the x-axis
                self.position = p + np.dot(rm, self.d)[:3]
                self.buffer[self.buffer_ptr] = tfs.euler_from_matrix(rm)[2]
                self.buffer_ptr = (self.buffer_ptr + 1) % self.buffer_size
                self.publish_filtered_tf(t.header)

    def publish_filtered_tf(self, header):
        yaw = np.median(self.buffer)
        q = tfs.quaternion_from_euler(0, 0, yaw - math.pi)
        ts = TransformStamped()
        ts.header = header
        ts.header.frame_id = self.source
        ts.child_frame_id = self.goal
        ts.transform.translation.x = self.position[0]
        ts.transform.translation.y = self.position[1]
        ts.transform.translation.z = 0
        ts.transform.rotation.x = q[0]
        ts.transform.rotation.y = q[1]
        ts.transform.rotation.z = q[2]
        ts.transform.rotation.w = q[3]
        msg = tfMessage()
        msg.transforms.append(ts)
        self.tf_pub.publish(msg)

    def publish_goal_cb(self, r):
        self.source = r.source_frame_id
        self.target = r.target_frame_id
        self.goal = r.goal_frame_id
        self.d = [r.displacement.x, r.displacement.y, r.displacement.z]
        return []


if __name__ == '__main__':
    rospy.init_node(NODE)
    filter_window_length = rospy.get_param('~filter_window_length', 30)
    tf_filter = TfFilter(filter_window_length)
    rospy.loginfo('Started [%s] node.' % NODE)
    rospy.spin()
