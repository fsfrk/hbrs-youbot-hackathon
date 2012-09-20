#!/usr/bin/env python

PACKAGE = 'raw_object_recognition'
NODE = 'collect_training_data'

import roslib
roslib.load_manifest(PACKAGE)
import sys
sys.path.append(roslib.packages.get_pkg_dir(PACKAGE) + '/common/src')
import argparse

import rospy
from smach import State, StateMachine, cb_interface
from smach_ros import ServiceState, ConditionState

from hbrs_srvs.srv import FindWorkspace
from hbrs_srvs.srv import AccumulateTabletopCloud
from hbrs_srvs.srv import ClusterTabletopCloud
from hbrs_srvs.srv import MakeBoundingBoxes, MakeBoundingBoxesRequest
from raw_srvs.srv import AnalyzeCloudColor

SERVICE_NAME = '/find_objects'

from confirm_state import ConfirmState


class StoreObject(State):
    def __init__(self, object_id):
        State.__init__(self,
                       outcomes=['stored'],
                       input_keys=['bounding_boxes', 'clusters'])
        self.dataset = Dataset(object_id)

    def execute(self, userdata):
        client = rospy.ServiceProxy('analyze_cloud_color', AnalyzeCloudColor)
        response = client(userdata.clusters[0])
        print 'Mean color', response.mean
        print 'Median color', response.median
        print 'Points', response.points
        #self.dataset.store(userdata.bounding_boxes[0].dimensions)
        return 'stored'


class Dataset:
    def __init__(self, id, rewrite=False):
        self.object_id = id
        mode = 'w' if rewrite else 'a'
        self.file = open('common/data/training/%s.txt' % id, mode)
        if rewrite:
            self.file.write('# object_id x y z point color\n')

    def __del__(self):
        self.file.close()

    def store(self, dim, color, pts, clouds):
        self.file.write('%s %.4f %.4f %.4f\n' % (self.object_id, dim.x,  dim.y,
                                                 dim.z))


if __name__ == '__main__':
    rospy.init_node(NODE)
    parser = argparse.ArgumentParser(description='''
    Collect training data for object recognition.
    To store sample press Enter, to drop press any other key.
    ''')
    parser.add_argument('object_id', help='id of the object (as in database)')
    parser.add_argument('--rewrite', action='store_true', help='delete any \
                        existing data for this object')
    args = parser.parse_args()
    sm = StateMachine(['succeeded', 'aborted', 'preempted'])
    with sm:
        StateMachine.add('FIND_WORKSPACE',
                         ServiceState('find_workspace',
                                      FindWorkspace,
                                      response_slots=['polygon']),
                         transitions={'succeeded': 'CONFIRM_WORKSPACE',
                                      'aborted': 'FIND_WORKSPACE'})
        StateMachine.add('CONFIRM_WORKSPACE',
                         ConfirmState('Is the detected workspace correct?'),
                         transitions={'yes': 'ACCUMULATE_CLOUD',
                                      'no': 'FIND_WORKSPACE'})
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
                         transitions={'succeeded': 'IS_ONLY_ONE_OBJECT'})
        StateMachine.add('IS_ONLY_ONE_OBJECT',
                         ConditionState(cond_cb=lambda userdata:
                                        len(userdata.clusters) == 1,
                                        input_keys=['clusters']),
                         transitions={'true': 'MAKE_BOUNDING_BOXES',
                                      'false': 'ACCUMULATE_CLOUD'})

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
                         transitions={'succeeded': 'CONFIRM_OBJECT'})
        StateMachine.add('CONFIRM_OBJECT',
                         ConfirmState('Is the detected object correct?'),
                         transitions={'yes': 'STORE_OBJECT',
                                      'no': 'ACCUMULATE_CLOUD'})
        StateMachine.add('STORE_OBJECT',
                         StoreObject(args.object_id),
                         transitions={'stored': 'ACCUMULATE_CLOUD'})
    rospy.loginfo('Starting data collection...')
    outcome = sm.execute()
