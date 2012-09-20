#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_transportation_test_iros2012')
import rospy

import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

class select_object_to_be_grasped(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','no_more_objects'],
            input_keys=['recognized_objects'],
            output_keys=['recognized_objects', 'object_to_grasp'])
        
    def execute(self, userdata):
        
        if(len(userdata.recognized_objects) == 0):
            return 'no_more_objects'
        
        userdata.task_list[]
        userdata.object_to_grasp = userdata.recognized_objects.pop() 
                
        return 'succeeded'


class get_obj_poses_for_goal_configuration(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'configuration_poses_not_available'],
            input_keys=['task_spec','obj_goal_configuration_poses'],
            output_keys=['obj_goal_configuration_poses'])
        
    def execute(self, userdata):
        
        sss.move("gripper","open")
        print userdata.task_spec.object_config 
        
        if (not rospy.has_param("/script_server/arm/" + userdata.task_spec.object_config)):
            rospy.logerr("configuration <<" + userdata.task_spec.object_config + ">> NOT available on parameter server")
            return 'configuration_poses_not_available'
            
        pose_names = rospy.get_param("/script_server/arm/" + userdata.task_spec.object_config)

        print pose_names
        
        for pose_name in pose_names:
            print "cfg pose: ", pose_name
            userdata.obj_goal_configuration_poses.append((userdata.task_spec.object_config + "/" + pose_name))
    

        userdata.obj_goal_configuration_poses.sort()
        
                
        return 'succeeded'



