#!/usr/bin/python

import roslib; roslib.load_manifest('raw_final_iros2012')
import rospy

import smach
import smach_ros

# import of generic states
from generic_basic_states import *
from generic_navigation_states import *
from generic_state_machines import *
from generic_robocup_states import *

SM_OMNI = smach.StateMachine(outcomes=['overall_success'])

SM_OMNI.userdata.current_task_index = 0
SM_OMNI.userdata.task_list = [Bunch('S1', 'W', '1'),
                              Bunch('S2', 'E', '3')]

with SM_OMNI:
    smach.StateMachine.add('SELECT_POSE_TO_APPROACH', select_pose_to_approach(),
                           transitions={'pose_selected': 'MOVE_TO_LOCATION',
                                        'location_not_known': 'INCREMENT_TASK_INDEX'})

    smach.StateMachine.add('MOVE_TO_LOCATION', approach_pose(),
                           transitions={'succeeded': 'WAIT_DESIRED_DURATION',
                                        'failed': 'INCREMENT_TASK_INDEX'})

    smach.StateMachine.add('WAIT_DESIRED_DURATION', wait_for_desired_duration(),
                           transitions={'succeeded': 'INCREMENT_TASK_INDEX'})

    smach.StateMachine.add('INCREMENT_TASK_INDEX', increment_task_index(),
                           transitions={'succeeded': 'SELECT_POSE_TO_APPROACH',
                                        'no_more_tasks': 'overall_success'})
