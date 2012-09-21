#!/usr/bin/env python

PACKAGE = 'raw_object_detection'
NODE = 'object_detector'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from smach import State, StateMachine, CBState, cb_interface
from smach_ros import ServiceState

from hbrs_srvs.srv import FindWorkspace
from hbrs_srvs.srv import AccumulateTabletopCloud
from hbrs_srvs.srv import ClusterTabletopCloud
from hbrs_srvs.srv import MakeBoundingBoxes, MakeBoundingBoxesRequest
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
            return 'failed'


@cb_interface(input_keys=['clusters', 'bounding_boxes', 'response'],
              output_keys=['response'],
              outcomes=['done'])
def pack_response(userdata):
    userdata.response = GetObjectsResponse()
    for c, b in zip(userdata.clusters, userdata.bounding_boxes):
        obj = Object()
        # TODO: expand the box to account for the fact that the bottom
        # 1 cm was cut off during cloud accumulation.
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
                         ServiceState('find_workspace',
                                      FindWorkspace,
                                      response_slots=['polygon']),
                         transitions={'succeeded': 'ACCUMULATE_CLOUD',
                                      'aborted': 'FIND_WORKSPACE_ABORTED'})
        StateMachine.add('FIND_WORKSPACE_ABORTED',
                         FindWorkspaceAborted(),
                         transitions={'retry': 'FIND_WORKSPACE',
                                      'failed': 'aborted'})
        StateMachine.add('ACCUMULATE_CLOUD',
                         ServiceState('accumulate_tabletop_cloud',
                                      AccumulateTabletopCloud,
                                      request_slots=['polygon'],
                                      response_slots=['cloud']),
                         transitions={'succeeded': 'CLUSTER_CLOUD'})
        StateMachine.add('CLUSTER_CLOUD',
                         ServiceState('cluster_tabletop_cloud',
                                      ClusterTabletopCloud,
                                      request_slots=['cloud', 'polygon'],
                                      response_slots=['clusters']),
                         transitions={'succeeded': 'MAKE_BOUNDING_BOXES'})

        @cb_interface(input_keys=['clusters', 'polygon'])
        def make_boxes_request_cb(userdata, request):
            r = MakeBoundingBoxesRequest()
            r.clouds = userdata.clusters
            r.axis.x = userdata.polygon.coefficients[0]
            r.axis.y = userdata.polygon.coefficients[1]
            r.axis.z = userdata.polygon.coefficients[2]
            return r

        StateMachine.add('MAKE_BOUNDING_BOXES',
                         ServiceState('make_bounding_boxes',
                                      MakeBoundingBoxes,
                                      request_cb=make_boxes_request_cb,
                                      input_keys=['clusters', 'polygon'],
                                      response_slots=['bounding_boxes']),
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
