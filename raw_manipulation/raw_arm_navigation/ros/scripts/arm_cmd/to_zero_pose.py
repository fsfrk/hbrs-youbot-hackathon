#!/usr/bin/python
import time

import roslib
roslib.load_manifest('raw_arm_navigation')
import rospy
import actionlib

from raw_script_server.msg import *
from simple_script_server import *

sss = simple_script_server()

def main():
    rospy.init_node('move_to_zero')
    sss.move("arm", "zeroposition")
    rospy.sleep(1)

## Main routine for running the script server
if __name__ == '__main__':
    main()
