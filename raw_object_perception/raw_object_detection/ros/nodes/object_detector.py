#!/usr/bin/env python

PACKAGE = 'raw_object_detection'
NODE = 'object_detector'

import roslib
roslib.load_manifest(PACKAGE)
import sys
from os.path import join

# Import states for calling services from hbrs_scene_segmentation
sys.path.append(join(roslib.packages.get_pkg_dir(PACKAGE), 'ros', 'src'))
import service_states

import rospy

from smach import State, StateMachine, CBState, cb_interface
from smach_ros import ServiceState

from hbrs_srvs.srv import GetObjects, GetObjectsResponse
from hbrs_msgs.msg import Object


class FindWorkspaceAborted(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['failed', 'retry'])
        self.retries = 3

    def execute(self, userdata):
        rospy.logwarn('Failed to find workspace, will attempt %i retries.' %
                      (self.retries))
        if self.retries > 0:
            self.retries -= 1
            return 'retry'
        else:
            self.retries = 3
            return 'failed'


@cb_interface(input_keys=['clusters', 'bounding_boxes', 'response'],
              output_keys=['response'],
              outcomes=['done'])
def pack_response(userdata):
    userdata.response = GetObjectsResponse()
    for c, b in zip(userdata.clusters, userdata.bounding_boxes):
        obj = Object()
        obj.dimensions.vector = b.dimensions
        obj.pose.pose.position = b.center
        obj.name = 'unknown'
        userdata.response.objects.append(obj)
    return 'done'


if __name__ == '__main__':
    rospy.init_node(NODE)
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'],
                      output_keys=['response'])
    with sm:
        StateMachine.add('FIND_WORKSPACE',
                         service_states.find_workspace,
                         transitions={'succeeded': 'ACCUMULATE_CLOUD',
                                      'aborted': 'FIND_WORKSPACE_ABORTED'})
        StateMachine.add('FIND_WORKSPACE_ABORTED',
                         FindWorkspaceAborted(),
                         transitions={'retry': 'FIND_WORKSPACE',
                                      'failed': 'aborted'})
        StateMachine.add('ACCUMULATE_CLOUD',
                         service_states.accumulate_tabletop_cloud,
                         transitions={'succeeded': 'CLUSTER_CLOUD'})
        StateMachine.add('CLUSTER_CLOUD',
                         service_states.cluster_tabletop_cloud,
                         transitions={'succeeded': 'MAKE_BOUNDING_BOXES'})
        StateMachine.add('MAKE_BOUNDING_BOXES',
                         service_states.make_bounding_boxes,
                         transitions={'succeeded': 'PACK_RESPONSE'})
        StateMachine.add('PACK_RESPONSE',
                         CBState(pack_response),
                         transitions={'done': 'succeeded'})

    def detect_objects_cb(request):
        rospy.loginfo('Received [detect_objects] request.')
        outcome = sm.execute()
        if outcome == 'succeeded':
            return sm.userdata.response
        else:
            raise rospy.ServiceException('Object detection failed')

    s = rospy.Service('detect_objects', GetObjects, detect_objects_cb)
    rospy.loginfo('Started [detect_objects] service.')
    rospy.spin()
