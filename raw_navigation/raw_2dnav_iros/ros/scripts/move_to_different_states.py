#!/usr/bin/python
#Class select
import roslib; roslib.load_manifest('raw_move_to_different_states')
import rospy

import smach
import smach_ros

class get_basic_navigation_task(smach.State):
    def __init__(self):
           smach.State.__init__(self, 
            outcomes=['task_received','wront_task_format'])


# The select_pose_to_approach function has been taken from the package raw_fetch_and_carry.
class select_pose_to_approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['pose_selected'],
            input_keys=['base_pose_list', 'base_pose_to_approach'],
            output_keys=['base_pose_list', 'base_pose_to_approach'])

    def execute(self, userdata):
        if(userdata.base_pose_to_approach == -1):
            userdata.base_pose_to_approach = userdata.base_pose_list[0]
        else if:
            count = 0
            for item in userdata.base_pose_list:
                if userdata.base_pose_to_approach == item:
                    userdata.base_pose_to_approach = userdata.base_pose_list[((count+1) % len(userdata.base_pose_list))]
                    break;
                count = count + 1 
        
        rospy.loginfo("selected pose: %s", userdata.base_pose_to_approach)
                
        return 'pose_selected'

class approach_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
    def execute(self, userdata):
        if(userdata.base_pose_to_approach = OdometryPose)
            return 'succeeded'
        else:
            return 'failed'

class wait_for_desired_duration(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['wait_time'])
    def execute(self, userdata):
	if(userdata.wait_time = 10)
            return 'succeeded' 

class increment_task_index(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['no_more_tasks'],
            input_keys=['base_pose_list', 'base_pose_to_approach'])
    def execute(self, userdata):
         if (userdata.base_pose_list != endOfList)  #To be replaced
           return('userdata.base_pose_list++')
         else if:
           return('no_more_tasks')

class move_to_exit(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'failed'])
    def execute(self, userdata):
         if (userdata.base_pose_list == endOfList)  #To be replaced
           return('succeeded')
         else if:
           return('failed')



