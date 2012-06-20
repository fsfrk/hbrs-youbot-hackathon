#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_manipulation_test_wc2012')
import rospy

import smach
import smach_ros

# generic states
from generic_robocup_states import *
from generic_navigation_states import *
from generic_manipulation_states import *
from generic_perception_states import *

from basic_manipulation_test_wc2012_states import *

# main
def main():
    rospy.init_node('basic_manipulation_test')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])
    
    # world knowledge
    SM.userdata.task_spec = 0
    
    SM.userdata.base_pose_to_approach = 0
    
    SM.userdata.recognized_objects = []
    SM.userdata.object_to_grasp = 0
    
    SM.userdata.rear_platform_free_poses = ['pltf_pose_1', 'pltf_pose_2', 'pltf_pose_3']
    SM.userdata.rear_platform_occupied_poses = []
    
    # open the container
    with SM:
        # add states to the container
        
        # move to the source pose, recognize objects, grasp them and put them on the rear platform
        smach.StateMachine.add('GET_TASK', get_basic_manipulation_task(),
            transitions={'task_received':'SELECT_SOURCE_POSE', 
                         'wront_task_format':'GET_TASK'})
        
        smach.StateMachine.add('SELECT_SOURCE_POSE', select_pose("source_pose"),
            transitions={'succeeded':'ADJUST_POSE_WRT_PLATFORM'})
        
        '''
        smach.StateMachine.add('APPROACH_SOURCE_POSE', approach_pose(),
            transitions={'succeeded':'ADJUST_POSE_WRT_PLATFORM'},
                        {'failed':'APPROACH_SOURCE_POSE'})
        '''

        smach.StateMachine.add('ADJUST_POSE_WRT_PLATFORM', adjust_pose_wrt_platform(),
            transitions={'succeeded':'MOVE_ARM_OUT_OF_VIEW',
                        'failed':'ADJUST_POSE_WRT_PLATFORM'})

        smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
            transitions={'succeeded':'ENABLE_PERCEPTION',
                        'failed':'MOVE_ARM_OUT_OF_VIEW'})
        
        smach.StateMachine.add('ENABLE_PERCEPTION', enable_object_finder(),
                transitions={'succeeded':'RECOGNIZE_OBJECTS',  
                             'failed':'ENABLE_PERCEPTION'})
        
        #ToDo: implement state
        smach.StateMachine.add('RECOGNIZE_OBJECTS', recognize_objects(),
            transitions={'succeeded':'SELECT_RECOGNIZED_OBJECT',
                        'failed':'RECOGNIZE_OBJECTS'})
        
        smach.StateMachine.add('SELECT_RECOGNIZED_OBJECT', select_recognized_object(),
            transitions={'succeeded':'PLACE_BASE_IN_FRONT_OF_OBJECT',
                        'no_more_objects':'SELECT_DESTINATION_POSE'})
        
        #ToDo: implement state
        smach.StateMachine.add('PLACE_BASE_IN_FRONT_OF_OBJECT', adjust_pose_wrt_recognized_obj(),
            transitions={'succeeded':'overall_success',
                        'failed':'PLACE_BASE_IN_FRONT_OF_OBJECT'})

        '''
        smach.StateMachine.add('GRASP_OBJ_WITH_VISUAL_SERVERING', grasp_obj_with_visual_servering(),
            transitions={'succeeded':'overall_success',
                        'failed':'MOVE_ARM_OUT_OF_VIEW'})
        
        smach.StateMachine.add('PLACE_OBJ_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
            transitions={'succeeded':'SELECT_OBJ_TO_BE_RECOGNIZED',
                        'failed':'PLACE_OBJ_ON_REAR_PLATFORM'})
        '''
        
        
        # go to the destination pose and place the objects in the desired configuration on the platform
        smach.StateMachine.add('SELECT_DESTINATION_POSE', select_pose("destination_pose"),
            transitions={'succeeded':'MOVE_TO_DESTINATION_POSE'})
        
        smach.StateMachine.add('MOVE_TO_DESTINATION_POSE', approach_pose(),
            transitions={'succeeded':'overall_success',
                        'failed':'overall_success'})
        
        '''
        smach.StateMachine.add('GRASP_OBJECT_FROM_PLTF', grasp_obj_from_pltf(),
            transitions={'succeeded':'overall_success',
                        'failed':'APPROACH_SOURCE_POSE'})
        
        smach.StateMachine.add('COMPUTE_OBJ_GOAL_POSITIONS', compute_obj_goal_positions(),
            transitions={'succeeded':'SELECT_OBJ_GOAL_POSE'})
        
        smach.StateMachine.add('SELECT_OBJ_GOAL_POSE', select_obj_goal_pose(),
            transitions={'succeeded':'GRASP_OBJ_FROM_PLTF'
                        'no_more_objects':'MOVE_TO_EXIT'})
        
        smach.StateMachine.add('GRASP_OBJ_FROM_PLTF', grasp_obj_from_pltf(),
            transitions={'succeeded':'PLACE_OBJ_ON_GOAL_POSE'})
        
        smach.StateMachine.add('PLACE_OBJ_ON_GOAL_POSE', place_obj_on_goal_pose(),
            transitions={'succeeded':'SELECT_OBJ_GOAL_POSE'})
        '''
            
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
