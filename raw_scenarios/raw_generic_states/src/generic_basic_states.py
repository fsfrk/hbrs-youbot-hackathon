#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')

import raw_srvs.srv
import rospy
import smach
import smach_ros
import commands
import os

from simple_script_server import *
sss = simple_script_server()

class init_robot(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
    def execute(self, userdata):
        
        # init arm
        arm_to_init = sss.move("arm", "initposition", blocking=False)
        
        #init gripper
        gripper_open = sss.move("gripper", "open")
                
        arm_to_init.wait();
        gripper_open.wait();
        
        rospy.loginfo("robot initialized")
        
        return 'succeeded'
    
    
class wait_for_open_door(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):

        door_client = rospy.ServiceProxy('/raw_door_status/door_status', raw_srvs.srv.ReturnBool)
        rospy.wait_for_service('/raw_door_status/door_status',20) # todo error handling

        # wait for open door
        door_open = False
        while not door_open:
            print "door open?:", door_open
            try:
                res = door_client()
                door_open = res.value
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                door_open = False
            
            rospy.sleep(0.3)

        # sleep here to give the referees a chance to open the door completly
        rospy.sleep(1)
        
        return 'succeeded'
