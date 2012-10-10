#!/usr/bin/python
import roslib; roslib.load_manifest('raw_final_iros2012')
import rospy

import smach
import smach_ros

# import of generic states
from final_iros2012_states import *


# main
def main():
    rospy.init_node('final')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed', 'missing_service'])

   

    with SM:
        smach.StateMachine.add('INIT_ROBOT', grasp_bin(),
                               transitions={'succeeded': 'overall_success',
                                            'open_drawer_poses_not_available':'overall_failed'})

       
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
