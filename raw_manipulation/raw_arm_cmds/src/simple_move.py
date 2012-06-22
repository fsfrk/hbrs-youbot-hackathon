#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_cmds')
import rospy
import brics_actuator.msg

from brics_actuator.msg import JointPositions, JointValue, Poison

import sys, select, termios, tty, signal

if __name__=="__main__":
    
    pub = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions)
    
    rospy.init_node('simple_arm_gripper_position')

    rospy.sleep(0.5)
    
    try:
        jp = JointPositions()
        
        jv1 = JointValue()
        jv1.joint_uri = "arm_joint_1"
        jv1.value = 0.011
        jv1.unit = "rad"
        
        jv2 = JointValue()
        jv2.joint_uri = "arm_joint_2"
        jv2.value = 0.011
        jv2.unit = "rad"

        jv3 = JointValue()
        jv3.joint_uri = "arm_joint_3"
        jv3.value = -0.016
        jv3.unit = "rad"
        
        jv4 = JointValue()
        jv4.joint_uri = "arm_joint_4"
        jv4.value = 0.023
        jv4.unit = "rad"
        
        jv5 = JointValue()
        jv5.joint_uri = "arm_joint_5"
        jv5.value = 0.12
        jv5.unit = "rad"
        

        p = Poison()
        jp.poisonStamp = p
        
        jp.positions = [jv1, jv2, jv3, jv4, jv5]
       
        pub.publish(jp)
        pub.publish(jp)

        
    except Exception, e:
        print e
'''
JOINT LIMITS:
[ WARN] [1340337373.846618553]: Cannot set arm joint 1: 
 The setpoint angle is out of range. The valid range is between 0.0100692 rad and 5.84014 rad and it is: 0.01 rad
[ WARN] [1340337373.847057559]: Cannot set arm joint 2: 
 The setpoint angle is out of range. The valid range is between 0.0100692 rad and 2.61799 rad and it is: 0.01 rad
[ WARN] [1340337373.847447567]: Cannot set arm joint 3: 
 The setpoint angle is out of range. The valid range is between -5.02655 rad and -0.015708 rad and it is: -0.01 rad
[ WARN] [1340337373.853084080]: Cannot set arm joint 4: 
 The setpoint angle is out of range. The valid range is between 0.0221239 rad and 3.4292 rad and it is: 0.01 rad
[ WARN] [1340337373.854206454]: Cannot set arm joint 5: 
 The setpoint angle is out of range. The valid range is between 0.110619 rad and 5.64159 rad and it is: 0.01 rad
'''
