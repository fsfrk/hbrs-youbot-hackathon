#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_manipulation_test_wc2012')
import rospy

import smach
import smach_ros

# generic states
from generic_robocup_states import *
#from generic_manipulation_states import *

# main
def main():
    rospy.init_node('basic_manipulation_test')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])
    
    # world knowledge
    SM.userdata.current_task_index = 0
    SM.userdata.task_list = []
    
    # open the container
    with SM:
        # add states to the container
        
        smach.StateMachine.add('GET_TASK', get_basic_manipulation_task(),
            transitions={'task_received':'overall_success', 
                         'wront_task_format':'overall_failed'})
        


            
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
