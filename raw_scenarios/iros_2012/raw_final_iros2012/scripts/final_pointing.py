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

SM_POINTING = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])

SM_POINTING.userdata.workstation = "D1"

with SM_POINTING:
        smach.StateMachine.add('MOVE_TO_WORKSTATION', approach_pose(),
                               remapping={'base_pose_to_approach': 'workstation'},
                               transitions={'succeeded': 'ADJUST_POSE_WRT_WORKSPACE',
                                            'failed': 'overall_failed'})

        smach.StateMachine.add('ADJUST_POSE_WRT_WORKSPACE', adjust_pose_wrt_workspace(),
                               transitions={'succeeded': 'MOVE_ARM_OUT_OF_VIEW',
                                            'failed': 'ADJUST_POSE_WRT_WORKSPACE'}) # TODO: recovery behavior here?

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
                               transitions={'succeeded': 'RECOGNIZE_OBJECTS'})

        smach.StateMachine.add('RECOGNIZE_OBJECTS', recognize_objects(),
                               transitions={'found_objects': 'POINT_AND_ANNOUNCE_OBJECTS',
                                            'no_objects_found': 'overall_failed',
                                            'srv_call_failed': 'overall_failed'}) # TODO: recovery!

        smach.StateMachine.add('POINT_AND_ANNOUNCE_OBJECTS', point_and_announce_objects(),
                               transitions={'succeeded': 'MOVE_BACKWARD'})

        smach.StateMachine.add('MOVE_BACKWARD', move_base_rel(-0.25, 0),
                               transitions={'succeeded': 'overall_success'})

