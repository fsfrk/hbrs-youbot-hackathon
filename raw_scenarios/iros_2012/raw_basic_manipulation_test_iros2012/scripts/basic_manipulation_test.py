#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_manipulation_test_iros2012')
import rospy

import smach
import smach_ros

# generic states
from generic_basic_states import *
from generic_robocup_states import *
from generic_navigation_states import *
from generic_manipulation_states import *
from generic_perception_states import *

from basic_manipulation_test_iros2012_states import *

# main
def main():
    rospy.init_node('basic_manipulation_test')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])
    
    # world knowledge
    SM.userdata.task_spec = 0
    
    SM.userdata.base_pose_to_approach = 0
    
    SM.userdata.recognized_objects = []
    SM.userdata.object_to_grasp = 0
    
    SM.userdata.rear_platform_free_poses = ['platform_right', 'platform_centre', 'platform_left']
    SM.userdata.rear_platform_occupied_poses = []
    
    SM.userdata.obj_goal_configuration_poses = []
    
    # open the container
    with SM:
        # add states to the container
        
        # move to the source pose, recognize objects, grasp them and put them on the rear platform
        smach.StateMachine.add('GET_TASK', get_basic_manipulation_task(),
            transitions={'task_received':'INIT_ROBOT', 
                         'wront_task_format':'GET_TASK'})
        
        smach.StateMachine.add('INIT_ROBOT', init_robot(),
            transitions={'succeeded':'SELECT_SOURCE_POSE'})

        smach.StateMachine.add('SELECT_SOURCE_POSE', select_base_pose("source_pose"),
            transitions={'succeeded':'APPROACH_SOURCE_POSE'})
            #transitions={'succeeded':'ADJUST_POSE_WRT_WORKSPACE'})
        
        smach.StateMachine.add('APPROACH_SOURCE_POSE', approach_pose(),
            transitions={'succeeded':'ADJUST_POSE_WRT_WORKSPACE',
                        'failed':'APPROACH_SOURCE_POSE'})
	
        smach.StateMachine.add('ADJUST_POSE_WRT_WORKSPACE', adjust_pose_wrt_workspace(),
            transitions={'succeeded':'MOVE_ARM_OUT_OF_VIEW',
                        'failed':'ADJUST_POSE_WRT_WORKSPACE'})

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
            transitions={'succeeded':'RECOGNIZE_OBJECTS'})
                
        smach.StateMachine.add('RECOGNIZE_OBJECTS', recognize_objects(),
            transitions={'found_objects':'SELECT_RECOGNIZED_OBJECT',
                        'srv_call_failed':'RECOGNIZE_OBJECTS',
                        'no_objects_found':'RECOGNIZE_OBJECTS'})
        
        smach.StateMachine.add('SELECT_RECOGNIZED_OBJECT', select_recognized_object_bmt(),
            transitions={'succeeded':'PLACE_BASE_IN_FRONT_OF_OBJECT',
                        'no_more_objects':'SELECT_DESTINATION_POSE'})
        
        smach.StateMachine.add('PLACE_BASE_IN_FRONT_OF_OBJECT', place_base_in_front_of_object(),
            transitions={'succeeded':'GRASP_OBJ_WITH_VISUAL_SERVERING',
            #transitions={'succeeded':'PLACE_OBJ_ON_REAR_PLATFORM',
                        'srv_call_failed':'PLACE_BASE_IN_FRONT_OF_OBJECT'},
            remapping={'object_pose':'object_to_grasp'})   

        smach.StateMachine.add('GRASP_OBJ_WITH_VISUAL_SERVERING', grasp_obj_with_visual_servering(),
            transitions={'succeeded':'PLACE_OBJ_ON_REAR_PLATFORM',
                        'failed':'GRASP_OBJ_WITH_VISUAL_SERVERING',
                        'vs_timeout':'SELECT_RECOGNIZED_OBJECT'})
        
        smach.StateMachine.add('PLACE_OBJ_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
            transitions=#{'succeeded':'SELECT_DESTINATION_POSE',
                        {'succeeded':'SELECT_RECOGNIZED_OBJECT', 
                        'no_more_free_poses':'SELECT_DESTINATION_POSE'})
                        #'no_more_free_poses':'overall_success'})

        # go to the destination pose and place the objects in the desired configuration on the platform
        smach.StateMachine.add('SELECT_DESTINATION_POSE', select_base_pose("destination_pose"),
            transitions={'succeeded':'APPROACH_DESTINATION_POSE'})
            #transitions={'succeeded':'overall_success'})
        
        smach.StateMachine.add('APPROACH_DESTINATION_POSE', approach_pose(),
            transitions={'succeeded':'ADJUST_POSE_WRT_WORKSPACE2',
                        'failed':'APPROACH_DESTINATION_POSE'})

        smach.StateMachine.add('ADJUST_POSE_WRT_WORKSPACE2', adjust_pose_wrt_workspace(),
            transitions={'succeeded':'GET_OBJ_POSES_FOR_CONFIGURATION',
                        'failed':'ADJUST_POSE_WRT_WORKSPACE2'})
        
        smach.StateMachine.add('GET_OBJ_POSES_FOR_CONFIGURATION', get_obj_poses_for_goal_configuration(),
            transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF',
                         'configuration_poses_not_available':'overall_failed'})
        
        smach.StateMachine.add('GRASP_OBJECT_FROM_PLTF', grasp_obj_from_pltf(),
            transitions={'succeeded':'PLACE_OBJ_IN_CONFIGURATION',
                        'no_more_obj_on_pltf':'SELECT_FINAL_POSE'})
        
        smach.StateMachine.add('PLACE_OBJ_IN_CONFIGURATION', place_object_in_configuration(),
            transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF',
                        'no_more_cfg_poses':'SELECT_FINAL_POSE'})
               
        # if everything is done move to final pose
        smach.StateMachine.add('SELECT_FINAL_POSE', select_base_pose("final_pose"),
            transitions={'succeeded':'MOVE_ARM_TO_INIT'})

        #smach.StateMachine.add('MOVE_ARM_TO_ZERO', move_arm("zeroposition"),
        #    transitions={'succeeded':'MOVE_ARM_TO_INIT'})

        smach.StateMachine.add('MOVE_ARM_TO_INIT', move_arm("initposition", do_blocking = False),
            transitions={'succeeded':'APPROACH_EXIT_POSE'})
        
        smach.StateMachine.add('APPROACH_EXIT_POSE', approach_pose("S2"),
            transitions={'succeeded':'overall_success',
                         'failed':'APPROACH_EXIT_POSE'})
                
       
    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('BASIC_MANIPULATION_TEST', SM, 'BASIC_MANIPULATION_TEST')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
