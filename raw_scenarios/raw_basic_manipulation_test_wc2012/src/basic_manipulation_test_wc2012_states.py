#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_manipulation_test_wc2012')
import rospy

import smach
import smach_ros


class select_pose(smach.State):
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

