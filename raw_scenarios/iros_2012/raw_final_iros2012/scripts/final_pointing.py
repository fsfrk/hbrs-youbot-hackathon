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

#def construct_push_bin_sm():
    #sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    #with sm:
        #smach.StateMachine.add('PUSH_BIN_DUMMY', do_nothing())
    #return sm

SM_POINTING = smach.StateMachine(outcomes=['overall_success', 'overall_failed', 'missing_service'])

SM_POINTING.userdata.workstation = "S2"

with SM_POINTING:
        #smach.StateMachine.add('MOVE_TO_WORKSTATION', approach_pose(),
                               #remapping={'base_pose_to_approach': 'workstation'},
                               #transitions={'succeeded': 'ADJUST_POSE_WRT_PLATFORM',
                                            #'failed': 'overall_failed'})

        #smach.StateMachine.add('ADJUST_POSE_WRT_PLATFORM', adjust_pose_wrt_platform(),
                               #transitions={'succeeded': 'MOVE_ARM_OUT_OF_VIEW',
                                            #'failed': 'ADJUST_POSE_WRT_PLATFORM'}) # TODO: recovery behavior here?

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
                               transitions={'succeeded': 'RECOGNIZE_OBJECTS'})

        smach.StateMachine.add('RECOGNIZE_OBJECTS', recognize_objects(),
                               transitions={'succeeded': 'POINT_AND_ANNOUNCE_OBJECTS',
                                            'failed': 'overall_failed'})

        smach.StateMachine.add('POINT_AND_ANNOUNCE_OBJECTS', point_and_announce_objects(),
                               transitions={'succeeded': 'MOVE_ARM_IN_SAFE_POS',
                                            'failed': 'POINT_AND_ANNOUNCE_OBJECTS'})

        smach.StateMachine.add('MOVE_ARM_IN_SAFE_POS', move_arm("initposition"),
                               transitions={'succeeded': 'overall_success'})
