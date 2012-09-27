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

from hbrs_srvs.srv import GetObjects, GetObjectsResponse
from hbrs_msgs.msg import Object
from raw_srvs.srv import RecognizeObject


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


class RecognizeObjects(State):
    def __init__(self):
        State.__init__(self,
                       input_keys=['clusters', 'bounding_boxes', 'names'],
                       output_keys=['names'],
                       outcomes=['done'])
        try:
            rospy.wait_for_service('recognize_object', timeout=5)
            self.recognize = rospy.ServiceProxy('recognize_object',
                                                RecognizeObject)
        except rospy.ROSException:
            rospy.logwarn('Object recognition service is not available, '
                          'will return "unknown" for all objects.')
            self.recognize = None

    def execute(self, ud):
        ud.names = list()
        for c, b in zip(ud.clusters, ud.bounding_boxes):
            if not self.recognize is None:
                response = self.recognize(c, b.dimensions)
                name = response.name
            else:
                name = 'unknown'
            ud.names.append(name)
        return 'done'


@cb_interface(input_keys=['clusters', 'bounding_boxes', 'names', 'response'],
              output_keys=['response'],
              outcomes=['done'])
def pack_response(ud):
    ud.response = GetObjectsResponse()
    for c, b, n in zip(ud.clusters, ud.bounding_boxes, ud.names):
        obj = Object()
        obj.dimensions.vector = b.dimensions
        obj.pose.pose.position = b.center
        obj.name = n
        ud.response.objects.append(obj)
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
                         transitions={'succeeded': 'RECOGNIZE_OBJECTS'})
        StateMachine.add('RECOGNIZE_OBJECTS',
                         RecognizeObjects(),
                         transitions={'done': 'PACK_RESPONSE'})
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
