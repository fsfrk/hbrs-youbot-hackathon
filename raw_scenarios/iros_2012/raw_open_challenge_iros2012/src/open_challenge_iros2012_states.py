#!/usr/bin/python
import roslib; roslib.load_manifest('raw_open_challenge_iros2012')
import rospy

import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()


