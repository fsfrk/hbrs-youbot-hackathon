#!/usr/bin/python


import roslib; roslib.load_manifest('raw_final_iros2012')
import rospy

import smach
import smach_ros

# import of generic states
from generic_basic_states import *
from generic_navigation_states import *
from generic_state_machines import *

from final_iros2012_states import *

def sm_pull_bin():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'], input_keys=['selected_marker', 'goal_pose'], output_keys=['selected_marker', 'goal_pose'])
    with sm:
                

        smach.StateMachine.add('CALCULATE_GOAL_POSE', calculate_goal_pose(),
                               transitions={'succeeded': 'ADJUST_BASE_TO_MARKER'},
                                remapping={'marker_pose':'selected_marker'})

        smach.StateMachine.add('ADJUST_BASE_TO_MARKER', place_base_in_front_of_object(),
                               transitions={'succeeded': 'GRASP_BIN',
                                            'srv_call_failed': 'failed'},
                               remapping={'object_pose':'goal_pose'})

        smach.StateMachine.add('GRASP_BIN', grasp_bin(),
                               transitions={'succeeded': 'PULL_BIN_OUT',
                                            'open_drawer_poses_not_available': 'failed'})

        smach.StateMachine.add('PULL_BIN_OUT', move_base_rel(BIN_PULL_DISTANCE, 0),
                               transitions={'succeeded': 'MOVE_ARM_TO_PREGRASP'})

        smach.StateMachine.add('MOVE_ARM_TO_PREGRASP', move_arm("pregrasp_laying_mex"),
                               transitions={'succeeded': 'succeeded'})
       
    return sm

SM_BINS = smach.StateMachine(outcomes=['overall_success', 'overall_failed', 'missing_service'])

SM_BINS.userdata.source_bin_id = "B1"
SM_BINS.userdata.task_workstation = "D1"
SM_BINS.userdata.source_workstation = "S1"
SM_BINS.userdata.destination_workstation = "S3"
SM_BINS.userdata.selected_marker = 0
SM_BINS.userdata.goal_pose = 0
SM_BINS.userdata.object_to_grasp = 0 # grasp_obj_with_visual_servoing

SM_BINS.userdata.task_marker_id = 0 # wait_for_task_marker
SM_BINS.userdata.detected_marker = 0 # detect_marker_at_source
SM_BINS.userdata.selected_marker = 0 # select_marker_to_approach_at_source
SM_BINS.userdata.visual_servoing_timeout_counter = 0 # increase_counter
SM_BINS.userdata.rear_platform_free_poses = ['platform_centre'] #place_obj_on_rear_pltf
SM_BINS.userdata.rear_platform_occupied_poses = [] #place_obj_on_rear_pltf

BIN_PULL_DISTANCE = 0.1

with SM_BINS:
        smach.StateMachine.add('MOVE_TO_TASK_WORKSTATION', approach_pose(),
                               remapping={'base_pose_to_approach': 'task_workstation'},
                               transitions={'succeeded': 'WAIT_FOR_TASK_MARKER',
                                            'failed': 'overall_failed'})

        smach.StateMachine.add('WAIT_FOR_TASK_MARKER', wait_for_task_marker(),
                               transitions={'found_marker': 'MOVE_TO_SOURCE_WORKSTATION'})

        smach.StateMachine.add('MOVE_TO_SOURCE_WORKSTATION', approach_pose(),
                               remapping={'base_pose_to_approach': 'source_workstation'},
                               transitions={'succeeded': 'MOVE_ARM_OUT_OF_VIEW_AT_SOURCE',
                                            'failed': 'overall_failed'}) # TODO: what does failure mean? how to recover?

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW_AT_SOURCE', move_arm_out_of_view(),
                               transitions={'succeeded': 'DETECT_MARKER_AT_SOURCE'})

        smach.StateMachine.add('DETECT_MARKER_AT_SOURCE', detect_marker(),
                               transitions={'found_marker': 'SELECT_MARKER_TO_APPROACH_AT_SOURCE',
                                            'no_marker_found': 'DETECT_MARKER_AT_SOURCE',
                                            'srv_call_failed':'overall_failed'})
        
        smach.StateMachine.add('SELECT_MARKER_TO_APPROACH_AT_SOURCE', select_marker_to_approach_at_source(),
                               transitions={'succeeded': 'GRASP_AND_PULL_OUT_SOURCE_BIN',
                                            'failed':'DETECT_MARKER_AT_SOURCE'})


        smach.StateMachine.add('GRASP_AND_PULL_OUT_SOURCE_BIN', sm_pull_bin(),
                               remapping={'': ''},
                               transitions={'succeeded': 'overall_failed',
                                            'failed': 'MOVE_ARM_TO_PREGRASP'}) # TODO: what does failure mean? how to recover?
        

        smach.StateMachine.add('MOVE_ARM_TO_PREGRASP', move_arm("pregrasp_laying_mex"),
                               transitions={'succeeded': 'MOVE_OVER_BIN_AT_SOURCE'})

        smach.StateMachine.add('MOVE_OVER_BIN_AT_SOURCE', move_base_rel(0.1, 0),
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
                               transitions={'succeeded': 'MOVE_ARM_IN_SAFE_POS',
                                            'no_more_free_poses': 'overall_failed'}) # TODO: could it happen?

        smach.StateMachine.add('MOVE_ARM_IN_SAFE_POS', move_arm("initposition"),
                               transitions={'succeeded': 'MOVE_TO_DESTINATION_WORKSTATION'})

        smach.StateMachine.add('MOVE_TO_DESTINATION_WORKSTATION', approach_pose(),
                               remapping={'base_pose_to_approach': 'destination_workstation'},
                               transitions={'succeeded': 'MOVE_ARM_OUT_OF_VIEW_AT_DESTINATION',
                                            'failed': 'overall_failed'})

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW_AT_DESTINATION', move_arm_out_of_view(),
                               transitions={'succeeded': 'DETECT_MARKER_AT_DESTINATION'})

        smach.StateMachine.add('DETECT_MARKER_AT_DESTINATION', detect_marker(),
                               transitions={'found_marker': 'SELECT_MARKER_TO_APPROACH_AT_DESTINATION',
                                            'no_marker_found': 'DETECT_MARKER_AT_DESTINATION',
                                            'srv_call_failed':'overall_failed'})

        smach.StateMachine.add('SELECT_MARKER_TO_APPROACH_AT_DESTINATION', select_marker_to_approach_at_destination(),
                               transitions={'succeeded': 'GRASP_AND_PULL_OUT_DESTINATION_BIN',
                                            'failed':'DETECT_MARKER_AT_DESTINATION'})

        smach.StateMachine.add('GRASP_AND_PULL_OUT_DESTINATION_BIN', sm_pull_bin(),
                               transitions={'succeeded': 'GRASP_OBJECT_FROM_PLATFORM',
                                            'failed': 'overall_failed'}) # TODO: what does failure mean? how to recover?

        smach.StateMachine.add('GRASP_OBJECT_FROM_PLATFORM', grasp_obj_from_pltf(),
                               transitions={'succeeded': 'PLACE_OBJECT_IN_BIN',
                                            'no_more_obj_on_pltf': 'PLACE_OBJECT_IN_BIN'})

        smach.StateMachine.add('MOVE_OVER_BIN_AT_DESTINATION', move_base_rel(0.1, 0),
                               transitions={'succeeded': 'PLACE_OBJECT_IN_BIN'})

        smach.StateMachine.add('PLACE_OBJECT_IN_BIN', place_object_in_bin(),
                               transitions={'succeeded': 'MOVE_ARM_IN_SAFE_POS2'})

        smach.StateMachine.add('MOVE_ARM_IN_SAFE_POS2', move_arm("initposition"),
                               transitions={'succeeded': 'overall_success'})
