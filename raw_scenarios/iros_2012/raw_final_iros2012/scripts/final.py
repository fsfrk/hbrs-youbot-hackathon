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


# main
def main():
    rospy.init_node('final')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed', 'missing_service'])

    # world knowledge
    SM.userdata.object_list = [];
    SM.userdata.rear_platform_free_poses = ['platform_centre'] # put the only object to be grasped in the centre of the platform
    SM.userdata.rear_platform_occupied_poses = []

    SM.userdata.source_bin_id = "B1"
    SM.userdata.source_workstation = "S1"
    SM.userdata.destination_workstation = "D1"

    with SM:
        smach.StateMachine.add('INIT_ROBOT', init_robot(),
                               transitions={'succeeded': 'WAIT_FOR_TASK_MARKER'})

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
                               transitions={'succeeded': 'PULL_BIN_OUT_AT_SOURCE'})

        smach.StateMachine.add('PULL_BIN_OUT_AT_SOURCE', pull_bin(),
                               #transitions={'succeeded': 'MOVE_TO_DESTINATION_WORKSTATION'})
                               transitions={'succeeded': 'overall_success'})
        '''
        smach.StateMachine.add('PULL_DRAWER_OUT_AT_SOURCE_WS', move_base_rel(-distance_to_workstation, 0),
            transitions={'succeeded':'MOVE_ARM_TO_PREGRASP'})
        
        smach.StateMachine.add('MOVE_ARM_TO_PREGRASP', move_arm("pregrasp_laying_mex"),
            transitions={'succeeded':'MOVE_OVER_DRAWER'})
        
        smach.StateMachine.add('MOVE_OVER_DRAWER', move_base_rel(0.05, 0),
            transitions={'succeeded':'GRASP_OBJECT_FROM_DRAWER'})
                           
        smach.StateMachine.add('GRASP_OBJECT_FROM_DRAWER', grasp_obj_with_visual_servering(),
            transitions={'succeeded':'PLACE_OBJECT_ON_REAR_PLATFORM', 
                         'failed':'GRASP_OBJECT_FROM_DRAWER'})
                                
        smach.StateMachine.add('PLACE_OBJECT_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
            transitions={'succeeded':'SM_GRASP_DRAWER_AT_SOURCE_TO_PUSH', 
                        'no_more_free_poses':'SM_GRASP_DRAWER_AT_SOURCE_TO_PUSH'})
        
        smach.StateMachine.add('SM_GRASP_DRAWER_AT_SOURCE_TO_PUSH', sm_grasp_drawer(),
             transitions={'drawer_grasped':'PUSH_DRAWER_IN_AT_SOURCE_WS',
                        'drawer_not_found':'SM_GRASP_DRAWER_AT_SOURCE_TO_PUSH',
                        'base_placement_failed':'SM_GRASP_DRAWER_AT_SOURCE_TO_PUSH'})
        
        smach.StateMachine.add('PUSH_DRAWER_IN_AT_SOURCE_WS', move_base_rel(distance_to_workstation, 0),
            transitions={'succeeded':'MOVE_ARM_IN_SAFE_POS'})
        
        smach.StateMachine.add('MOVE_ARM_IN_SAFE_POS', move_arm("initposition"),
            transitions={'succeeded':'MOVE_BACK_FIXED_DISTANCE'})

        smach.StateMachine.add('MOVE_BACK_FIXED_DISTANCE', move_base_rel(-distance_to_workstation,0),
            transitions={'succeeded':'MOVE_TO_DESTINATION_WORKSTATION'})

'''

        smach.StateMachine.add('MOVE_TO_DESTINATION_WORKSTATION', approach_pose(),
                               remapping={'base_pose_to_approach': 'destination_workstation'},
                               transitions={'succeeded': 'ADJUST_POSE_WRT_BIN_AT_DESTINATION',
                                            'failed': 'overall_failed'})

        smach.StateMachine.add('ADJUST_POSE_WRT_BIN_AT_DESTINATION', adjust_pose_wrt_bin(),
                               remapping={'bin_marker_id': 'destination_bin_id'},
                               transitions={'succeeded': 'GRASP_BIN_AT_DESTINATION',
                                            'failed': 'MOVE_TO_SOURCE_WORKSTATION'})

        smach.StateMachine.add('GRASP_BIN_AT_DESTINATION', grasp_bin(),
                               transitions={'succeeded': 'PULL_BIN_OUT_AT_DESTINATION'})

        smach.StateMachine.add('PULL_BIN_OUT_AT_DESTINATION', pull_bin(),
                               transitions={'succeeded': 'PLACE_OBJECT_IN_BIN'})

        #smach.StateMachine.add('GRASP_OBJECT_FROM_PLTF', grasp_obj_from_pltf(),
            #transitions={'succeeded':'PLACE_OBJECT_IN_DRAWER',
                        #'no_more_obj_on_pltf':'PLACE_OBJECT_IN_DRAWER'})

        smach.StateMachine.add('PLACE_OBJECT_IN_BIN', place_object_in_bin(),
                               transitions={'succeeded': 'MOVE_ARM_IN_SAFE_POS2'})

        smach.StateMachine.add('MOVE_ARM_IN_SAFE_POS2', move_arm("initposition"),
                               transitions={'succeeded': 'MOVE_TO_EXIT'})

        smach.StateMachine.add('MOVE_TO_EXIT', approach_pose("EXIT"),
                               transitions={'succeeded': 'overall_success',
                                            'failed':'overall_failed'})

    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('FINAL', SM, 'FINAL')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
