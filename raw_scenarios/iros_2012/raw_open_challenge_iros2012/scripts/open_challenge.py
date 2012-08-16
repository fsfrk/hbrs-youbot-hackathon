#!/usr/bin/python
import roslib; roslib.load_manifest('raw_open_challenge_iros2012')
import rospy

import smach
import smach_ros

# import of generic states
#from generic_robocup_states import *


# main
def main():
    rospy.init_node('open_challenge')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])
    
    # world knowledge
    #SM.userdata.task_spec = 0
    
    
    # open the container
    with SM:
        # add states to the container
        
     
        
       
    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('OPEN_CHALLENGE', SM, 'OPEN_CHALLENGE')
    smach_viewer.start()

    SM.execute()

    # stop SMACH viewer
    rospy.spin()
    # smach_thread.stop()
    smach_viewer.stop()

if __name__ == '__main__':
    main()