#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_navigation_test_wc2012')
import rospy

import smach
import smach_ros

# generic states
from generic_robocup_states import *
from generic_navigation_states import *

# main
def main():
    rospy.init_node('basic_navigation_test')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])
    
    # world knowledge
    SM.userdata.current_task_index = 0
    SM.userdata.task_list = []
    
    # open the container
    with SM:
        # add states to the container
        
        smach.StateMachine.add('GET_TASK', get_basic_navigation_task(),
            transitions={'task_received':'SELECT_POSE_TO_APPROACH', 
                         'wront_task_format':'GET_TASK'})
        
        smach.StateMachine.add('SELECT_POSE_TO_APPROACH', select_pose_to_approach(),
            transitions={'pose_selected':'WAIT_DESIRED_DURATION',
                         'location_not_known':'INCREMENT_TASK_INDEX'})
        
        smach.StateMachine.add('MOVE_TO_LOCATION', approach_pose(),
            transitions={'succeeded':'WAIT_DESIRED_DURATION', 
                        'failed':'INCREMENT_TASK_INDEX'})
        
        smach.StateMachine.add('WAIT_DESIRED_DURATION', wait_for_desired_duration(),
            transitions={'succeeded':'INCREMENT_TASK_INDEX'})
        
        smach.StateMachine.add('INCREMENT_TASK_INDEX', increment_task_index(),
            transitions={'succeeded':'SELECT_POSE_TO_APPROACH',
                         'no_more_tasks':'overall_success'})


            
    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('BASIC_NAVIGATION_TEST', SM, 'BASIC_NAVIGATION_TEST')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
