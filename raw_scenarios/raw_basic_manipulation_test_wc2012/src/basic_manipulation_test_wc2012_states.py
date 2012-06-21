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
            outcomes=['succeeded', 'configuration_poses_not_available'],
            input_keys=['task_spec'],
            output_keys=['obj_goal_configuration_poses'])
        
    def execute(self, userdata):
        
        print userdata.task_spec.configuration 
        
        if (not rospy.has_param("/script_server/arm/ + " userdata.task_spec.configuration):
            rospy.logerr("configuration <<" + userdata.task_spec.configuration + ">> NOT available on parameter server")
            return 'configuration_poses_not_available'
            
        pose_names = rospy.get_param(/script_server/arm/ + " userdata.task_spec.configuration)
        
        for pose in pose_names
            userdata.obj_goal_configuration_poses.append(pose)
                
        return 'succeeded'






