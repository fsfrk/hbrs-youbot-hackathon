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

def construct_push_bin_sm():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        smach.StateMachine.add('PUSH_BIN_DUMMY', do_nothing())
    return sm

SM_BINS = smach.StateMachine(outcomes=['overall_success', 'overall_failed', 'missing_service'])

SM_BINS.userdata.source_bin_id = "B1"
SM_BINS.userdata.task_workstation = "S1"
SM_BINS.userdata.source_workstation = "S1"
SM_BINS.userdata.destination_workstation = "S1"

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
                               transitions={'succeeded': 'ADJUST_POSE_WRT_BIN_AT_SOURCE',
                                            'failed': 'overall_failed'}) # TODO: what does failure mean? how to recover?

        smach.StateMachine.add('ADJUST_POSE_WRT_BIN_AT_SOURCE', adjust_pose_wrt_bin(),
                               remapping={'bin_marker_id': 'source_bin_id'},
                               transitions={'succeeded': 'GRASP_BIN_AT_SOURCE',
                                            'failed': 'MOVE_TO_SOURCE_WORKSTATION'})

        smach.StateMachine.add('GRASP_BIN_AT_SOURCE', grasp_bin(),
                               transitions={'succeeded': 'PULL_BIN_OUT_AT_SOURCE',
                                            'open_drawer_poses_not_available': 'overall_failed'})

        smach.StateMachine.add('PULL_BIN_OUT_AT_SOURCE', move_base_rel(BIN_PULL_DISTANCE, 0),
                               transitions={'succeeded': 'MOVE_ARM_TO_PREGRASP'})

        smach.StateMachine.add('MOVE_ARM_TO_PREGRASP', move_arm("pregrasp_laying_mex"),
                               transitions={'succeeded': 'MOVE_OVER_BIN'})

        smach.StateMachine.add('MOVE_OVER_BIN', move_base_rel(0.05, 0),
                               transitions={'succeeded': 'GRASP_OBJECT_FROM_BIN'})

        smach.StateMachine.add('GRASP_OBJECT_FROM_BIN', grasp_obj_with_visual_servering(),
                               transitions={'succeeded': 'PLACE_OBJECT_ON_REAR_PLATFORM',
                                            'vs_timeout': 'GRASP_OBJECT_FROM_BIN',
                                            'failed': 'GRASP_OBJECT_FROM_BIN'}) # TODO: is it a proper recovery behavior?

        smach.StateMachine.add('PLACE_OBJECT_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
                               transitions={'succeeded': 'PUSH_BIN_AT_SOURCE',
                                            'no_more_free_poses': 'PUSH_BIN_AT_SOURCE'}) # TODO: could it happen?

        smach.StateMachine.add('PUSH_BIN_AT_SOURCE', construct_push_bin_sm(),
                               transitions={'succeeded': 'MOVE_ARM_IN_SAFE_POS',
                                            'failed': 'overall_failed'})

        smach.StateMachine.add('MOVE_ARM_IN_SAFE_POS', move_arm("initposition"),
                               transitions={'succeeded': 'MOVE_TO_DESTINATION_WORKSTATION'})

        smach.StateMachine.add('MOVE_TO_DESTINATION_WORKSTATION', approach_pose(),
                               remapping={'base_pose_to_approach': 'destination_workstation'},
                               transitions={'succeeded': 'ADJUST_POSE_WRT_BIN_AT_DESTINATION',
                                            'failed': 'overall_failed'})

        smach.StateMachine.add('ADJUST_POSE_WRT_BIN_AT_DESTINATION', adjust_pose_wrt_bin(),
                               remapping={'bin_marker_id': 'destination_bin_id'},
                               transitions={'succeeded': 'GRASP_BIN_AT_DESTINATION',
                                            'failed': 'MOVE_TO_DESTINATION_WORKSTATION'})

        smach.StateMachine.add('GRASP_BIN_AT_DESTINATION', grasp_bin(),
                               transitions={'succeeded': 'PULL_BIN_OUT_AT_DESTINATION',
                                            'open_drawer_poses_not_available': 'overall_failed'})

        smach.StateMachine.add('PULL_BIN_OUT_AT_DESTINATION', move_base_rel(BIN_PULL_DISTANCE, 0),
                               transitions={'succeeded': 'GRASP_OBJECT_FROM_PLATFORM'})

        smach.StateMachine.add('GRASP_OBJECT_FROM_PLATFORM', grasp_obj_from_pltf(),
                               transitions={'succeeded': 'PLACE_OBJECT_IN_BIN',
                                            'no_more_obj_on_pltf': 'PLACE_OBJECT_IN_BIN'})

        smach.StateMachine.add('PLACE_OBJECT_IN_BIN', place_object_in_bin(),
                               transitions={'succeeded': 'MOVE_ARM_IN_SAFE_POS2'})

        smach.StateMachine.add('MOVE_ARM_IN_SAFE_POS2', move_arm("initposition"),
                               transitions={'succeeded': 'overall_success'})
