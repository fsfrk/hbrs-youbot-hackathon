#!/usr/bin/python


import roslib; roslib.load_manifest('raw_final_iros2012')
import rospy

import smach
import smach_ros

# import of generic states
from generic_basic_states import *
from generic_navigation_states import *
from generic_manipulation_states import *
from generic_state_machines import *

from final_iros2012_states import *

def construct_push_bin_sm():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        smach.StateMachine.add('PUSH_BIN_DUMMY', do_nothing())
    return sm


BIN_PULL_DISTANCE = -0.20

if __name__ == '__main__':
    rospy.init_node('bin_grasping_test')
    SM_BINS = smach.StateMachine(outcomes=['overall_success', 'overall_failed', 'missing_service'])
    SM_BINS.userdata.bin_id = 'drawer_1'
    SM_BINS.userdata.selected_marker = 0
    SM_BINS.userdata.detected_marker = []
    SM_BINS.userdata.goal_pose = 0
    SM_BINS.userdata.rear_platform_free_poses = ['platform_centre']
    SM_BINS.userdata.rear_platform_occupied_poses = []
    SM_BINS.userdata.visual_servoing_timeout_counter = 0
    SM_BINS.userdata.object_to_grasp = 0

    with SM_BINS:
        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
                               transitions={'succeeded': 'DETECT_MARKER'})

        smach.StateMachine.add('DETECT_MARKER', detect_marker(),
                               transitions={'found_marker': 'SELECT_MARKER_TO_APPROACH',
                                            'no_marker_found': 'DETECT_MARKER',
                                            'srv_call_failed':'overall_failed'})
        
        smach.StateMachine.add('SELECT_MARKER_TO_APPROACH', select_marker_to_approach(),
                               transitions={'succeeded': 'CALCULATE_GOAL_POSE',
                                            'failed':'DETECT_MARKER'})

        smach.StateMachine.add('CALCULATE_GOAL_POSE', calculate_goal_pose(),
                               transitions={'succeeded': 'ADJUST_BASE_TO_MARKER'},
                                remapping={'marker_pose':'selected_marker'})

        smach.StateMachine.add('ADJUST_BASE_TO_MARKER', place_base_in_front_of_object(),
                               transitions={'succeeded': 'GRASP_BIN',
                                            'srv_call_failed': 'overall_failed'},
                               remapping={'object_pose':'goal_pose'})

        smach.StateMachine.add('GRASP_BIN', grasp_bin(),
                               transitions={'succeeded': 'PULL_BIN_OUT',
                                            'open_drawer_poses_not_available': 'overall_failed'})

        smach.StateMachine.add('PULL_BIN_OUT', move_base_rel(BIN_PULL_DISTANCE, 0),
                               transitions={'succeeded': 'MOVE_ARM_TO_PREGRASP'})

        smach.StateMachine.add('MOVE_ARM_TO_PREGRASP', move_arm("pregrasp_laying_mex"),
                               transitions={'succeeded': 'MOVE_OVER_BIN'})

        smach.StateMachine.add('MOVE_OVER_BIN', move_base_rel(0.1, 0),
                               transitions={'succeeded': 'GRASP_OBJECT_FROM_BIN'})

        smach.StateMachine.add('GRASP_OBJECT_FROM_BIN', grasp_obj_with_visual_servering(),
                               transitions={'succeeded': 'PLACE_OBJECT_ON_REAR_PLATFORM',
                                            'vs_timeout': 'INCREASE_VISUAL_SERVOING_TIMEOUT_COUNTER',
                                            'failed': 'INCREASE_VISUAL_SERVOING_TIMEOUT_COUNTER'}) # TODO: is it a proper recovery behavior?
        
        smach.StateMachine.add('INCREASE_VISUAL_SERVOING_TIMEOUT_COUNTER', increase_counter(3),
                               transitions={'succeeded': 'GRASP_OBJECT_FROM_BIN',
                                            'counter_exceeded_limit': 'overall_failed'},
                               remapping={'counter': 'visual_servoing_timeout_counter'}) 
        

        smach.StateMachine.add('PLACE_OBJECT_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
                               transitions={'succeeded': 'overall_success',
                                            'no_more_free_poses': 'overall_failed'}) # TODO: could it happen?



    SM_BINS.execute()
    rospy.spin()
