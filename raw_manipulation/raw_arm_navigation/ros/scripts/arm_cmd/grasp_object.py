#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_navigation')

import rospy

def callback(msg):
    

def main():
    rospy.init_node('grasp_object')
    self.received_state = False
    if (not rospy.has_param("joints")):
        rospy.logerr("No arm joints given.")
        exit(0)
    else:
        self.joint_names = sorted(rospy.get_param("joints"))
        rospy.loginfo("arm joints: %s", self.joint_names)
    # read joint limits
    self.joint_limits = []
    for joint in self.joint_names:
        if ((not rospy.has_param("limits/" + joint + "/min")) or (not rospy.has_param("limits/" + joint + "/min"))):
            rospy.logerr("No arm joint limits given.")
            exit(0)
        else:
            limit = arm_navigation_msgs.msg.JointLimits()
            limit.joint_name = joint 
            limit.min_position = rospy.get_param("limits/" + joint + "/min")
            limit.max_position = rospy.get_param("limits/" + joint + "/max")
            self.joint_limits.append(limit)
    self.current_joint_configuration = [0 for i in range(len(self.joint_names))]
    self.unit = "rad"
		
    # subscriptions
    rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)

if __name__ == '__main__':
    main()

'''
NEW LAYING PREGRASP: include head
name: ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'gripper_finger_joint_l', 'gripper_finger_joint_r']
position: [2.9496232178824551, 2.3698483351867927, -2.2653867545403319, 2.7554864916852222, 2.470110409722865, 0.0, 0.0]

NEW LAYING GRASP: head rotation will be set by Matt.
name: ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'gripper_finger_joint_l', 'gripper_finger_joint_r']
position: [2.9496332870896782, 2.4775888524733656, -2.269360869247123, 2.7519909167608056, 2.4698891708035982, 0.0, 0.0]
'''
