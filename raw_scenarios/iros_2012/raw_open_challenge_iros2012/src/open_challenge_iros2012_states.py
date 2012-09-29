#!/usr/bin/python
import roslib; roslib.load_manifest('raw_open_challenge_iros2012')
import rospy

import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

class point_to_recognized_objects(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_list', 'recognized_objects'])

        
    def execute(self, userdata):
       #sss.move("gripper", "open", blocking=False)
       # sss.move("arm", "zeroposition")
        
        for object in userdata.recognized_objects:         
            
            # ToDo: need to be adjusted to correct stuff           
            if object.pose.pose.position.z <= 0.0 or object.pose.pose.position.z >= 0.10:
                continue
    
            global planning_mode
            sss.move("arm", "zeroposition", mode=planning_mode)                             

            #object.pose.pose.position.z = object.pose.pose.position.z + 0.02
            object.pose.pose.position.x = object.pose.pose.position.x + 0.01
            object.pose.pose.position.y = object.pose.pose.position.y - 0.005

            #ToDo Check the 'pregrasp' position to point the object
            handle_arm = sss.move("arm", [object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z, "/base_link"], mode=planning_mode)

            if handle_arm.get_state() == 3:
                #sss.move("gripper", "close", blocking=False)
                #ToDo complete the sss.play with the objects names from recognized_objects and add .wav files to the directory
                #sss.play(object.name)
                rospy.sleep(3.0)
                sss.move("arm", "zeroposition", mode=planning_mode)        
                return 'succeeded'    
            else:
                rospy.logerr('could not find IK for current object')

        return 'failed'


