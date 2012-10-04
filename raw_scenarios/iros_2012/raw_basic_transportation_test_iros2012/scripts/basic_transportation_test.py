#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_transportation_test_iros2012')
import rospy

import smach
import smach_ros

# import of generic states
# generic states
from generic_basic_states import *
from generic_robocup_states import *
from generic_navigation_states import *
from generic_manipulation_states import *
from generic_perception_states import *
from basic_transportation_test_iros2012_states import *


# main
def main():
    rospy.init_node('basic_transportation_test')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])
    
    # world knowledge
    SM.userdata.task_list = []
    SM.userdata.base_pose_to_approach = 0
    SM.userdata.lasttask = Bunch(location="", obj_names="")
    SM.userdata.current_task_index = 0
    SM.userdata.recognized_objects = []
    SM.userdata.object_to_grasp = 0

    SM.userdata.rear_platform_free_poses = []
    SM.userdata.rear_platform_free_poses.append(Bunch(obj_name="", platform_pose='platform_right'))
    SM.userdata.rear_platform_free_poses.append(Bunch(obj_name="", platform_pose='platform_centre'))
    SM.userdata.rear_platform_free_poses.append(Bunch(obj_name="", platform_pose='platform_left'))

    SM.userdata.rear_platform_occupied_poses = []
    SM.userdata.obj_goal_configuration_poses = []
    SM.userdata.destinaton_free_poses = []
    SM.userdata.source_visits = []
    
     # open the container
    with SM:
        
        smach.StateMachine.add('INIT_ROBOT', init_robot(),
            transitions={'succeeded':'GET_TASK'})

        smach.StateMachine.add('GET_TASK', get_basic_transportation_task(),
            transitions={'task_received':'SETUP_BTT', 
                         'wront_task_format':'GET_TASK'})
        
        smach.StateMachine.add('SETUP_BTT', setup_btt(),
            transitions={'success':'SELECT_SOURCE_SUBTASK'})
        
        smach.StateMachine.add('SELECT_SOURCE_SUBTASK', select_btt_subtask(type="source"),
            transitions={'task_selected':'MOVE_TO_SOURCE_LOCATION', 
                         'task_selected_but_already_in_this_pose':'ADJUST_POSE_WRT_TO_PLATFORM_AT_SOURCE',
                         'no_more_task_for_given_type':'SELECT_DELIVER_WORKSTATION'})

        smach.StateMachine.add('MOVE_TO_SOURCE_LOCATION', approach_pose(),
            transitions={'succeeded':'ADJUST_POSE_WRT_TO_PLATFORM_AT_SOURCE', 
                        'failed':'MOVE_TO_SOURCE_LOCATION'})

        smach.StateMachine.add('ADJUST_POSE_WRT_TO_PLATFORM_AT_SOURCE', adjust_pose_wrt_platform(),
            transitions={'succeeded':'MOVE_ARM_OUT_OF_VIEW',
                        'failed':'ADJUST_POSE_WRT_TO_PLATFORM_AT_SOURCE'}) 

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
            transitions={'succeeded':'RECOGNIZE_OBJECTS'})

        smach.StateMachine.add('RECOGNIZE_OBJECTS', recognize_objects(),
            transitions={'found_objects':'SELECT_OBJECT_TO_BE_GRASPED',
                        'no_objects_found':'RECOGNIZE_OBJECTS',
                        'srv_call_failed':'RECOGNIZE_OBJECTS'})

        smach.StateMachine.add('SELECT_OBJECT_TO_BE_GRASPED', select_object_to_be_grasped(),
            transitions={'obj_selected':'GRASP_OBJ_WITH_VISUAL_SERVERING',
                        'no_obj_selected':'SKIP_SOURCE_POSE',
                        'no_more_free_poses_at_robot_platf':'SELECT_DELIVER_WORKSTATION'})            

        smach.StateMachine.add('PLACE_BASE_IN_FRONT_OF_OBJECT', place_base_in_front_of_object(),
            transitions={'succeeded':'GRASP_OBJ_WITH_VISUAL_SERVERING',
                         'srv_call_failed':'PLACE_BASE_IN_FRONT_OF_OBJECT'},
            remapping={'object_pose':'object_to_be_grasped'})    

        smach.StateMachine.add('GRASP_OBJ_WITH_VISUAL_SERVERING', grasp_obj_with_visual_servering(),
            transitions={'succeeded':'PLACE_OBJ_ON_REAR_PLATFORM',
                        'failed':'GRASP_OBJ_WITH_VISUAL_SERVERING'})
           
        smach.StateMachine.add('PLACE_OBJ_ON_REAR_PLATFORM', place_obj_on_rear_platform_btt(),
            transitions={'succeeded':'SELECT_OBJECT_TO_BE_GRASPED',
                        'no_more_free_poses':'SELECT_DELIVER_WORKSTATION'})
        
        
        # MISC STATES
        smach.StateMachine.add('SKIP_SOURCE_POSE', skip_pose('source'),
            transitions={'pose_skipped':'SELECT_SOURCE_SUBTASK',
                         'pose_skipped_but_limit_reached':'SELECT_DELIVER_WORKSTATION'})  
        
        smach.StateMachine.add('SKIP_DESTINATION_POSE', skip_pose('destination'),
            transitions={'pose_skipped':'SELECT_DELIVER_WORKSTATION',
                         'pose_skipped_but_limit_reached':'SELECT_DELIVER_WORKSTATION'})  
        
        

        # DELIVERY
        smach.StateMachine.add('SELECT_DELIVER_WORKSTATION', select_delivery_workstation(),
            transitions={'success':'MOVE_TO_DESTINATION_LOCATION',
                         'no_more_dest_tasks':'MOVE_TO_EXIT'})
        
        smach.StateMachine.add('MOVE_TO_DESTINATION_LOCATION', approach_pose(),
            transitions={'succeeded':'ADJUST_POSE_WRT_TO_PLATFORM_AT_DESTINATION', 
                        'failed':'MOVE_TO_DESTINATION_LOCATION'})

        smach.StateMachine.add('ADJUST_POSE_WRT_TO_PLATFORM_AT_DESTINATION', adjust_pose_wrt_platform(),
            transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF',
                        'failed':'ADJUST_POSE_WRT_TO_PLATFORM_AT_DESTINATION'})  

        smach.StateMachine.add('GRASP_OBJECT_FROM_PLTF', grasp_obj_from_pltf_btt(),
            transitions={'object_grasped':'PLACE_OBJ_IN_CONFIGURATION',
                    'no_more_obj_for_this_workspace':'MOVE_ARM_TO_INIT'})
    
        smach.StateMachine.add('PLACE_OBJ_IN_CONFIGURATION', place_object_in_configuration_btt(),
            transitions={'succeeded':'GRASP_OBJECT_FROM_PLTF',
                    'no_more_cfg_poses':'MOVE_ARM_OUT_OF_VIEW_2'})

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW_2', move_arm_out_of_view(),
            transitions={'succeeded':'SELECT_DELIVER_WORKSTATION'})


        smach.StateMachine.add('MOVE_ARM_TO_INIT', move_arm("initposition"),
            transitions={'succeeded':'CHECK_IF_PLTF_HAS_STILL_OBJS'})

        smach.StateMachine.add('CHECK_IF_PLTF_HAS_STILL_OBJS', check_if_platform_has_still_objects(),
            transitions={'still_objs_on_robot_pltf':'SKIP_DESTINATION_POSE',
                         'no_more_objs_on_robot_pltf':'SELECT_SOURCE_SUBTASK'})


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
