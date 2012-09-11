#!/usr/bin/env python
'''
author:arjun
service that returns whether the gripper is closed or not as a boolean
'''
import roslib; roslib.load_manifest('raw_arm_navigation')
import rospy
import sensor_msgs.msg
from raw_srvs.srv import *

isGripperClosed  = None
def is_gripper_closed_server(): 
        
        rospy.init_node("is_gripper_closed_server")
        #is_gripper_closed service
        s  = rospy.Service('is_gripper_closed',  ReturnBool , handle_is_gripper_closed)
                
        # subscriptions
        rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, joint_states_callback)
          
     
def handle_is_gripper_closed(self):
    return isGripperClosed

def joint_states_callback(msg):
    #rospy.loginfo("gripper opening distance is" + str(float(msg.position[14])))
    if(float(msg.position[14]) <  0.003):
        global isGripperClosed 
        isGripperClosed = True
    else:
        isGripperClosed = False                
    return True
    
if __name__ == "__main__":
        
    is_gripper_closed_server()    
    rospy.loginfo("IsGripperClosed service started!")    
    rospy.spin()