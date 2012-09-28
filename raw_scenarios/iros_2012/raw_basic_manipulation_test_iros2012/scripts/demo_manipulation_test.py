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
    
    SM.userdata.object_list = [];
                                            # x, y, z, roll, pitch, yaw
    SM.userdata.rear_platform_free_poses = ['platform_centre']
    SM.userdata.rear_platform_occupied_poses = []
    
    SM.userdata.obj_goal_configuration_poses = []
    
    # open the container
    with SM:
        # add states to the container
        
        # move to the source pose, recognize objects, grasp them and put them on the rear platform

        
        smach.StateMachine.add('INIT_ROBOT', init_robot(),
            transitions={'succeeded':'ADJUST_POSE_WRT_PLATFORM'})

        smach.StateMachine.add('ADJUST_POSE', adjust_pose_wrt_platform(),
            transitions={'succeeded':'SM_GRASP_OBJECT',
                        'failed':'ADJUST_POSE'})
	
        smach.StateMachine.add('DEMO_GRASP_VERTICAL_OBJECT', demo_grasp_vertical_object(),
            transitions={'succeeded':'PLACE_OBJ_ON_REAR_PLATFORM',
                        'failed':'DEMO_GRASP_VERTICAL_OBJECT'})
        
       smach.StateMachine.add('PLACE_OBJECT_ON_REAR_PLATFORM', place_obj_on_rear_platform(),
            transitions={'succeeded':'MOVE_ARM_TO_INIT', 
                        'no_more_free_poses':'PLACE_OBJECT_ON_REAR_PLATFORM'})
               
        smach.StateMachine.add('MOVE_ARM_TO_INIT', move_arm("initposition", do_blocking = False),
            transitions={'succeeded':'overall_success',
                         'failed':'MOVE_ARM_TO_INIT'})
       
    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('DEMO_MANIPULATION_TEST', SM, 'DEMO_MANIPULATION_TEST')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
