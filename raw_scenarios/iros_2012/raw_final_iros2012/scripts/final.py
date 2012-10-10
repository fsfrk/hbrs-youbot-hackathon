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
from final_bins import SM_BINS
from final_omni import SM_OMNI
from final_pointing import SM_POINTING

def main():
    rospy.init_node('final')

    SM = smach.StateMachine(outcomes=['overall_success', 'overall_failed', 'missing_service'])

    # world knowledge
    SM.userdata.object_list = [];
    SM.userdata.rear_platform_free_poses = ['platform_centre'] # put the only object to be grasped in the centre of the platform
    SM.userdata.rear_platform_occupied_poses = []

    with SM:
        smach.StateMachine.add('INIT_ROBOT', init_robot(),
                               transitions={'succeeded': 'SM_BINS'})

        smach.StateMachine.add('SM_BINS', SM_BINS,
                               transitions={'overall_success': 'SM_POINTING',
                                            'overall_failed': 'MOVE_TO_EXIT'})

        smach.StateMachine.add('SM_POINTING', SM_POINTING,
                               transitions={'overall_success': 'SM_OMNI',
                                            'overall_failed': 'MOVE_TO_EXIT'})

        smach.StateMachine.add('SM_OMNI', SM_OMNI,
                               transitions={'overall_success': 'MOVE_TO_EXIT',
                                            'overall_failed': 'MOVE_TO_EXIT'})

        smach.StateMachine.add('MOVE_TO_EXIT', approach_pose("EXIT"),
                               transitions={'succeeded': 'overall_success',
                                            'failed':'overall_failed'})

    # Start SMACH viewer
    smach_viewer = smach_ros.IntrospectionServer('FINAL', SM, 'FINAL')
    smach_viewer.start()

    #SM.execute()
    #SM_BINS.execute()
    #SM_POINTING.execute()
    SM_OMNI.execute()

    # stop SMACH viewer
    rospy.spin()
    smach_viewer.stop()

if __name__ == '__main__':
    main()
