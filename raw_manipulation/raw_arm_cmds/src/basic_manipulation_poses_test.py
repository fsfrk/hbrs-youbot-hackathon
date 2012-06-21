#!/usr/bin/python
import time

import roslib
roslib.load_manifest('raw_arm_cmds')
import rospy
import actionlib

from simple_script_server import *

sss = simple_script_server()

## Main 
if __name__ == '__main__':
    rospy.init_node('basic_manipulation_poses_test')
    
    sss.move("arm","zeroposition")
    
    raw_input("Press Enter to got to platform left...")

    sss.move("arm","platform_intermediate")
    sss.move("arm","platform_left")
    
    raw_input("Press Enter to go to platform centre...")

    sss.move("arm","platform_intermediate")
    sss.move("arm","platform_centre")
    
    raw_input("Press Enter to go to platform right...")

    sss.move("arm","platform_intermediate")
    sss.move("arm","platform_right")
    
    raw_input("Press Enter to return to zero pose")

    sss.move("arm","platform_intermediate")
    sss.move("arm","zeroposition")
