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
    rospy.init_node('basic_zig_zag_test')
    
    sss.move("arm","zeroposition")
    
    raw_input("Press Enter to got to platform left...")

    #sss.move("arm","platform_intermediate")
    sss.move("arm","zz_one")
    
    raw_input("Press Enter to go to platform centre...")

    #sss.move("arm","platform_intermediate")
    sss.move("arm","zz_two")
    
    raw_input("Press Enter to go to platform right...")

    #sss.move("arm","platform_intermediate")
    sss.move("arm","zz_three")
    
    raw_input("Press Enter to return to zero pose")

    #sss.move("arm","platform_intermediate")
    sss.move("arm","zz_four")

    raw_input("press Enter..")
    sss.move("arm","zz_five")
    
    raw_input("press Enter to finish")
    sss.move("arm","zeroposition")
