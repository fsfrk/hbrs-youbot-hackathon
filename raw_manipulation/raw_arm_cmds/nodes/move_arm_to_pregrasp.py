#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_cmds')

import rospy

OBJECT_POSE_TOPIC = "object_pose"
OBJECT_POSE_MSG = 

class move_arm_to_pregrasp:
	
	def __init__(self):
        self.received_object_pose = False
        self.joint_namespace = "/arm_1/arm_controller/"
        if (not rospy.has_param(self.joint_namespace+"joints")):
            rospy.logerr("No arm joints given.")
            exit(0)
        else:
            self.joint_names = sorted(rospy.get_param(self.joint_namespace+"joints"))
            rospy.loginfo("arm joints: %s", self.joint_names)
        # read joint limits
        self.joint_limits = []
        for joint in self.joint_names:
            if ((not rospy.has_param(self.joint_namespace+"limits/" + joint + "/min")) or (not rospy.has_param(self.joint_namespace+"limits/" + joint + "/min"))):
                rospy.logerr("No arm joint limits given.")
                exit(0)
            else:
                limit = arm_navigation_msgs.msg.JointLimits()
                limit.joint_name = joint 
                limit.min_position = rospy.get_param(self.joint_namespace+"limits/" + joint + "/min")
                limit.max_position = rospy.get_param(self.joint_namespace+"limits/" + joint + "/max")
                self.joint_limits.append(limit)
        self.current_joint_configuration = [0 for i in range(len(self.joint_names))]
        self.unit = "rad"
            
        # subscriptions
        rospy.Subscriber(OBJECT_POSE_TOPIC, sensor_msgs.msg.JointState, self.object_pose_callback)
    
    def object_pose_callback(self, msg):
        #for k in range(len(self.joint_names)):
         #   for i in range(len(msg.name)):
         #       if (msg.name[i] == self.joint_names[k]):
                    #rospy.loginfo("%s: %f", msg.name[i], msg.position[i])
         #           self.current_joint_configuration[k] = msg.position[i]
        #print 'joint states received'
        self.received_object_pose = True
		
	def move(self, grasp_name):

def main():
	rospy.init_node("move_arm_to_pregrasp")

if __name__ == "__main__":
	main()
