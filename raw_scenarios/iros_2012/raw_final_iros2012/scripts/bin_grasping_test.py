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


BIN_PULL_DISTANCE = 0.1

if __name__ == '__main__':
    rospy.init_node('bin_grasping_test')
    SM_BINS = smach.StateMachine(outcomes=['overall_success', 'overall_failed', 'missing_service'])
    SM_BINS.userdata.bin_id = 'drawer_1'
    with SM_BINS:
        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
                               transitions={'succeeded': 'ADJUST_POSE_WRT_BIN'})

        smach.StateMachine.add('ADJUST_POSE_WRT_BIN', adjust_pose_wrt_bin(),
                               remapping={'bin_marker_id': 'bin_id'},
                               transitions={'succeeded': 'overall_success',
                                            'failed': 'overall_failed'})

        #smach.StateMachine.add('GRASP_BIN', grasp_bin(),
                               #transitions={'succeeded': 'PULL_BIN_OUT',
                                            #'open_drawer_poses_not_available': 'overall_failed'})

        #smach.StateMachine.add('PULL_BIN_OUT', move_base_rel(BIN_PULL_DISTANCE, 0),
                               #transitions={'succeeded': 'overall_success'})

        #smach.StateMachine.add('MOVE_ARM_TO_PREGRASP', move_arm("pregrasp_laying_mex"),
                               #transitions={'succeeded': 'MOVE_OVER_BIN'})

        #smach.StateMachine.add('MOVE_OVER_BIN', move_base_rel(0.05, 0),
                               #transitions={'succeeded': 'GRASP_OBJECT_FROM_BIN'})

        #smach.StateMachine.add('GRASP_OBJECT_FROM_BIN', grasp_obj_with_visual_servering(),
                               #transitions={'succeeded': 'PLACE_OBJECT_ON_REAR_PLATFORM',
                                            #'vs_timeout': 'GRASP_OBJECT_FROM_BIN',
                                            #'failed': 'GRASP_OBJECT_FROM_BIN'}) # TODO: is it a proper recovery behavior?

        #smach.StateMachine.add('PLACE_OBJECT_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
                               #transitions={'succeeded': 'PUSH_BIN_AT_SOURCE',
                                            #'no_more_free_poses': 'PUSH_BIN_AT_SOURCE'}) # TODO: could it happen?

        #smach.StateMachine.add('PUSH_BIN_AT_SOURCE', construct_push_bin_sm(),
                               #transitions={'succeeded': 'MOVE_ARM_IN_SAFE_POS',
                                            #'failed': 'overall_failed'})

        #smach.StateMachine.add('MOVE_ARM_IN_SAFE_POS', move_arm("initposition"),

    SM_BINS.execute()
    rospy.spin()
