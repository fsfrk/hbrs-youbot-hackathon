#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_transportation_test_iros2012')
import rospy

import smach
import smach_ros

# import of generic states
# generic states
from generic_robocup_states import *
from generic_navigation_states import *
from generic_manipulation_states import *
from generic_perception_states import *


# main
def main():
    rospy.init_node('basic_transportation_test')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])
    
    # world knowledge
    SM.userdata.task_list = []

    SM.userdata.base_pose_to_approach = 0

    SM.userdata.current_task_index = 0
    
    SM.userdata.recognized_objects = []

    SM.userdata.object_to_grasp = 0
    
    SM.userdata.rear_platform_free_poses = ['platform_right', 'platform_centre', 'platform_left']

    SM.userdata.rear_platform_occupied_poses = []
    
    SM.userdata.obj_goal_configuration_poses = []
    
     # open the container
    with SM:
        # add states to the container
        
        # Get the Transportation task spec
        smach.StateMachine.add('GET_TASK', get_basic_transportation_task(),
            transitions={'task_received':'overall_success', 
                         'wront_task_format':'GET_TASK'})

        # Navigate robot to desired location        
        smach.StateMachine.add('SELECT_POSE', select_pose_to_approach(),
            transitions={'pose_selected':'MOVE_TO_LOCATION',
                         'location_not_known':'INCREMENT_TASK_INDEX'})
        
        smach.StateMachine.add('MOVE_TO_LOCATION', approach_pose(),
            transitions={'succeeded':'WAIT_DESIRED_DURATION', 
                        'failed':'INCREMENT_TASK_INDEX'})

        smach.StateMachine.add('WAIT_DESIRED_DURATION', wait_for_desired_duration(),
            transitions={'succeeded':'ALIGN BASE WRT PLATFORM'})

        # Align the robot

        smach.StateMachine.add('ALIGN_BASE_WRT_PLATFORM', adjust_pose_wrt_platform(),
            transitions={'succeeded':'SELECT_TASK',
                        'failed':'ALIGN_BASE_WRT_PLATFORM'}) 

       # Select task : Fetch an object or Place an object

        smach.StateMachine.add('SELECT_TASK', select_task(),
            transitions={'fetch_object':'MOVE_ARM_OUT_OF_VIEW',
                        'place_object':'GET_OBJ_POSES_FOR_CONFIGURATION'})

       # States to Pick object from the platform 

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
            transitions={'succeeded':'RECOGNIZE_OBJECTS'})

        smach.StateMachine.add('RECOGNIZE_OBJECTS', recognize_objects(),
            transitions={'succeeded':'SELECT_RECOGNIZED_OBJECT',
                        'failed':'overall_failed'})
'''
        smach.StateMachine.add('SELECT_OBJECT_TO_BE_GRASPED', select_object_to_be_grasped(),
            transitions={'succeeded':'PLACE_BASE_IN_FRONT_OF_OBJECT',
                        'no_more_objects':'INCREMENT_TASK_INDEX'})            
'''
        smach.StateMachine.add('SELECT_RECOGNIZED_OBJECT', select_recognized_object(),
            transitions={'succeeded':'PLACE_BASE_IN_FRONT_OF_OBJECT',
                        'no_more_objects':'SELECT_DESTINATION_POSE'})

        smach.StateMachine.add('PLACE_BASE_IN_FRONT_OF_OBJECT', place_base_in_front_of_object(),
            transitions={'succeeded':'GRASP_OBJ_WITH_VISUAL_SERVERING'},
            remapping={'object_pose':'object_to_grasp'})    

        smach.StateMachine.add('GRASP_OBJ_WITH_VISUAL_SERVERING', grasp_obj_with_visual_servering(),
            transitions={'succeeded':'PLACE_OBJ_ON_REAR_PLATFORM',
                        'failed':'GRASP_OBJ_WITH_VISUAL_SERVERING'})
        '''
        smach.StateMachine.add('IS_OBJECT_GRASPED', is_object_grasped(),
            transitions={'obj_grasped':'PLACE_OBJ_ON_REAR_PLATFORM',
                        'obj_not_grasped':'GRASP_OBJ_WITH_VISUAL_SERVERING'})                
        ''''
        
        smach.StateMachine.add('PLACE_OBJ_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
            transitions={'succeeded':'SELECT_RECOGNIZED_OBJECT',
                        'no_more_free_poses':'INCREMENT_TASK_INDEX',
                        'failed':'PLACE_OBJ_ON_REAR_PLATFORM'})

       # States to Place object from the platform 

        smach.StateMachine.add('GET_OBJ_POSES_FOR_CONFIGURATION', get_obj_poses_for_goal_configuration(),
            transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF',
                     'configuration_poses_not_available':'overall_failed'})

        smach.StateMachine.add('GRASP_OBJECT_FROM_PLTF', grasp_obj_from_pltf(),
            transitions={'succeeded':'IS_OBJECT_GRASPED',
                    'no_more_obj_on_pltf':'MOVE_ARM_OUT_OF_VIEW2'})

        smach.StateMachine.add('IS_OBJECT_GRASPED', is_object_grasped(),
            transitions={'obj_grasped':'PLACE_OBJ_IN_CONFIGURATION',
                        'obj_not_grasped':'GRASP_OBJECT_FROM_PLTF'})   
    
        smach.StateMachine.add('PLACE_OBJ_IN_CONFIGURATION', place_object_in_configuration(),
            transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF',
                    'no_more_cfg_poses':'MOVE_ARM_OUT_OF_VIEW2'})

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW2', move_arm_out_of_view(),
            transitions={'succeeded':'INCREMENT_TASK_INDEX'})

       # Increment task index           
        smach.StateMachine.add('INCREMENT_TASK_INDEX', increment_task_index(),
            transitions={'succeeded':'SELECT_POSE_TO_APPROACH',
                         'no_more_tasks':'MOVE_TO_EXIT'})

       # Exit Robot 
        smach.StateMachine.add('MOVE_TO_EXIT', approach_pose("EXIT"),
            transitions={'succeeded':'overall_success', 
                        'failed':'MOVE_TO_EXIT'})

    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('BASIC_TRANSPORTATION_TEST', SM, 'BASIC_TRANSPORTATION_TEST')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
