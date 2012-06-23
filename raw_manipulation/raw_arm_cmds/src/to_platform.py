#!/usr/bin/python
import time

import roslib
roslib.load_manifest('raw_arm_cmds')
import rospy
import actionlib

from simple_script_server import *

sss = simple_script_server()

def move_to_platform(pose):
    sss.move("arm", "zeroposition")
    sss.move("arm", "platform_intermediate")
    
    if pose == 'l':
        sss.move("arm","platform_left_pre")
        sss.move("arm","platform_left")
        rospy.sleep(2.0)
        sss.move("arm","platform_left_pre")
    elif pose == 'c':
        sss.move("arm","platform_centre_pre")
        sss.move("arm","platform_centre")
        rospy.sleep(2.0)
        sss.move("arm","platform_centre_pre")
    elif pose == 'r':
        sss.move("arm","platform_right_pre")
        sss.move("arm","platform_right")
        rospy.sleep(2.0)
        sss.move("arm","platform_right_pre")
    else:
         rospy.logerr("not a platform pose")
         return
    
     rospy.sleep(1.0)
     sss.move("arm","platform_intermediate")
     sss.move("arm","zeroposition")
     return

def main():
    rospy.init_node('move_to_rear_platform')
    
    correct_input = False 
    
    which_platform_pose = ''
    
    while correct_input == False:
        which_platform_pose = raw_input("\nChoose your platform pose (l,c,r): "
        if which_platform_pose == 'l' || which_platform_pose == 'c' ||which_platform_pose == 'r':
            correct_input = True
        else:
            user_quit = raw_input("\nq to quit: ")
            if user_quit == 'q':
                return
    
    move_to_platform(which_platform_pose)
    
    print("done.")
    rospy.sleep(1)

## Main routine for running the script server
if __name__ == '__main__':
    main()
