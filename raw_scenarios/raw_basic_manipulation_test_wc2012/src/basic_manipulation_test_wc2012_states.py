#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_manipulation_test_wc2012')
import rospy

import smach
import smach_ros


class select_base_pose(smach.State):
    def __init__(self, pose):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['task_spec', 'base_pose_to_approach'],
            output_keys=['base_pose_to_approach'])
        
        self.pose = pose

    def execute(self, userdata):
        if(self.pose == 'source_pose'):
            userdata.base_pose_to_approach = userdata.task_spec.source_pose
        elif(self.pose == 'destination_pose'):
            userdata.base_pose_to_approach = userdata.task_spec.destination_pose
        elif(self.pose == 'final_pose'):
            userdata.base_pose_to_approach = userdata.task_spec.final_pose
        
        rospy.loginfo("selected pose: %s", userdata.base_pose_to_approach)
                
        return 'succeeded'


class select_recognized_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','no_more_objects'],
            input_keys=['recognized_objects'],
            output_keys=['recognized_objects', 'object_to_grasp'])
        
    def execute(self, userdata):
        
        if(len(userdata.recognized_objects) == 0):
            return 'no_more_objects'
        
        userdata.object_to_grasp = userdata.recognized_objects.pop() 
                
        return 'succeeded'


class get_obj_poses_for_goal_configuration(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['task_spec'],
            output_keys=['obj_goal_configuration_poses'])
        
    def execute(self, userdata):
        
        print userdata.task_spec.configuration 
        
        userdata.obj_goal_configuration_poses = []
                
        return 'succeeded'

class select_obj_goal_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['obj_goal_configuration_poses'],
            output_keys=['obj_goal_pose'])
        
    def execute(self, userdata):
        
        print userdata.obj_goal_configuration_poses.pop() 
        
        userdata.obj_goal_pose = []
                
        return 'succeeded'


class place_obj_on_goal_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['obj_goal_pose'])
        
    def execute(self, userdata):
        
        print userdata.obj_goal_pose 
    
                
        return 'succeeded'

