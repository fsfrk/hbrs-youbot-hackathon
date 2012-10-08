#!/usr/bin/python
import roslib; roslib.load_manifest('raw_open_challenge_iros2012')
import rospy

import smach
import smach_ros

# import of generic states
#from generic_robocup_states import *
from generic_basic_states import *
from generic_robocup_states import *
from generic_navigation_states import *
from generic_manipulation_states import *
from generic_perception_states import *

from open_challenge_test_iros2012_states import *
from basic_manipulation_test_iros2012_states import *




# main
def main():
    rospy.init_node('open_challenge')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])
    
    # world knowledge
    SM.userdata.base_pose_to_approach = 0
    SM.userdata.recognized_objects = []
    SM.userdata.object_to_grasp = 0
    
    SM.userdata.rear_platform_free_poses = ['platform_right', 'platform_centre', 'platform_left']
    SM.userdata.rear_platform_occupied_poses = []
    
    SM.userdata.obj_goal_configuration_poses = []
    
    # open the container
    with SM:
        # add states to the container
        
        smach.StateMachine.add('INIT_ROBOT', init_robot(),
            transitions={'succeeded':'SELECT_SOURCE_POSE'})
        
        smach.StateMachine.add('APPROACH_SOURCE_POSE', approach_pose("S1"),
            transitions={'succeeded':'ADJUST_POSE_WRT_PLATFORM',
                        'failed':'APPROACH_SOURCE_POSE'})
	
        smach.StateMachine.add('ADJUST_POSE_WRT_PLATFORM', adjust_pose_wrt_platform(),
            transitions={'succeeded':'MOVE_ARM_OUT_OF_VIEW',
                        'failed':'ADJUST_POSE_WRT_PLATFORM'})

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
            transitions={'succeeded':'RECOGNIZE_OBJECTS'})
                
        smach.StateMachine.add('RECOGNIZE_OBJECTS', recognize_objects(),
            transitions={'succeeded':'POINT_TO_RECOGNIZED_OBJECTS',
                        'failed':'overall_failed'})

        smach.StateMachine.add('POINT_TO_SELECTED_OBJECT', point_to_and_announce_recognized_objects(),
            transitions={'succeeded':'MOVE_ARM_TO_INIT',
                        'failed':'POINT_TO_SELECTED_OBJECT'})
        
            
        smach.StateMachine.add('MOVE_ARM_TO_INIT', move_arm("initposition", do_blocking = False),
            transitions={'succeeded':'APPROACH_EXIT_POSE'})
        
        smach.StateMachine.add('APPROACH_EXIT_POSE', approach_pose("EXIT"),
            transitions={'succeeded':'overall_success',
                         'failed':'APPROACH_EXIT_POSE'})
                  
        
       
    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('OPEN_CHALLENGE', SM, 'OPEN_CHALLENGE')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
