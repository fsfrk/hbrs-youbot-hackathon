#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_cmds')

import rospy
from simple_script_server import *
import sensor_msgs.msg

sss = simple_script_server()

class Grasper():
    def __init__(self):
        self.JOINT_TWO_LAYING_GRASP = 2.47758
        self.received_state = False
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
        rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)
    
    def joint_states_callback(self, msg):
        for k in range(len(self.joint_names)):
            for i in range(len(msg.name)):
                if (msg.name[i] == self.joint_names[k]):
                    #rospy.loginfo("%s: %f", msg.name[i], msg.position[i])
                    self.current_joint_configuration[k] = msg.position[i]
        #print 'joint states received'
        self.received_state = True
    
    def graspobject(self, grasp_name):
        if self.received_state == True:
            grasp_pose = []
            #if not len(self.current_joint_configuration) == len(self.joint_names):
                #print "not the same number of joints"
            #else: 
                #print "there are ",len(self.joint_names)," joints"
            for i in range(len(self.joint_names)):
                grasp_pose.append(self.current_joint_configuration[i])
                if grasp_name == "laying":
                    if i == 1:
                        grasp_pose[i] = self.JOINT_TWO_LAYING_GRASP
                else: 
                    rospy.logerr("undefined grasp")
                    grasp_pose[i] = self.current_joint_configuration[i]
            #print "the grasp pose is", grasp_pose
            rospy.set_param("/script_server/arm/grasp_laying_mex", grasp_pose)
        else:
            rospy.logerr("no joint values recieved")
            return
        if rospy.has_param("/script_server/arm/grasp_laying_mex"):
            sss.move("arm","grasp_laying_mex")
            rospy.delete_param('/script_server/arm/grasp_laying_mex')
            return
        
    
def main():
    rospy.init_node('grasp_object')
    grasper = Grasper()
    print("waiting 0.02 for arm joint values")
    rospy.sleep(0.05)
    grasper.graspobject("laying")
    print("did it work?")

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
