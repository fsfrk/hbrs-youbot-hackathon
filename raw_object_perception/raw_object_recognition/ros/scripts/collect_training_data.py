#!/usr/bin/env python

PACKAGE = 'raw_object_recognition'
NODE = 'collect_training_data'

import roslib
roslib.load_manifest(PACKAGE)
import sys
sys.path.append(roslib.packages.get_pkg_dir(PACKAGE) + '/common/src')
import argparse
import os

import rospy
from smach import State, StateMachine, cb_interface
from smach_ros import ServiceState, ConditionState

from hbrs_srvs.srv import FindWorkspace
from hbrs_srvs.srv import AccumulateTabletopCloud
from hbrs_srvs.srv import ClusterTabletopCloud
from hbrs_srvs.srv import MakeBoundingBoxes, MakeBoundingBoxesRequest
from raw_srvs.srv import AnalyzeCloudColor, AnalyzeCloudColorRequest

from confirm_state import ConfirmState
from dataset import Dataset


class StoreObject(State):
    def __init__(self, dataset, object_id):
        State.__init__(self,
                       outcomes=['stored'],
                       input_keys=['bounding_boxes', 'clusters', 'mean',
                                   'median', 'points'])
        base = roslib.packages.get_pkg_dir(PACKAGE)
        self.dataset = Dataset(os.path.join(base, 'common', 'data'), dataset)
        self.object_id = object_id

    def execute(self, ud):
        self.dataset.store(self.object_id, ud.bounding_boxes[0].dimensions,
                           ud.points, ud.mean, ud.median)
        return 'stored'


class Counter(State):
    def __init__(self, limit):
        State.__init__(self, outcomes=['trigger', 'pass'])
        self.limit = limit
        self.counter = 0

    def execute(self, userdata):
        self.counter += 1
        if self.counter == self.limit:
            self.counter = 0
            return 'trigger'
        else:
            return 'pass'

if __name__ == '__main__':
    rospy.init_node(NODE)
    parser = argparse.ArgumentParser(description='''
    Collect training data for object recognition.
    To store sample press Enter, to drop press any other key.
    ''')
    parser.add_argument('object_id', help='id of the object (as in database)')
    parser.add_argument('--dataset', help='dataset name (default "standard")',
                        default='standard')
    parser.add_argument('--confirm-every', help=('ask for confirmation after'
                        'this many successful calls to the object detection'
                        'service'), default=1)
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

        @cb_interface(input_keys=['bounding_boxes'],
                      output_keys=['bounding_boxes'])
        def make_boxes_response_cb(userdata, response):
            userdata.bounding_boxes = response.bounding_boxes
            for b in userdata.bounding_boxes:
                b.dimensions.x += 0.01

        StateMachine.add('MAKE_BOUNDING_BOXES',
                         ServiceState('make_bounding_boxes',
                                      MakeBoundingBoxes,
                                      request_cb=make_boxes_request_cb,
                                      response_cb=make_boxes_response_cb),
                         transitions={'succeeded': 'ANALYZE_CLOUD_COLOR'})

        @cb_interface(input_keys=['clusters'])
        def analyze_color_request_cb(userdata, request):
            r = AnalyzeCloudColorRequest()
            r.cloud = userdata.clusters[0]
            return r

        StateMachine.add('ANALYZE_CLOUD_COLOR',
                         ServiceState('analyze_cloud_color',
                                      AnalyzeCloudColor,
                                      request_cb=analyze_color_request_cb,
                                      response_slots=['mean',
                                                      'median',
                                                      'points']),
                         transitions={'succeeded': 'COUNTER'})
        StateMachine.add('COUNTER',
                         Counter(int(args.confirm_every)),
                         transitions={'trigger': 'CONFIRM_OBJECT',
                                      'pass': 'STORE_OBJECT'})
        StateMachine.add('CONFIRM_OBJECT',
                         ConfirmState('Is the detected object correct?'),
                         transitions={'yes': 'STORE_OBJECT',
                                      'no': 'ACCUMULATE_CLOUD'})
        StateMachine.add('STORE_OBJECT',
                         StoreObject(args.dataset, args.object_id),
                         transitions={'stored': 'ACCUMULATE_CLOUD'})
    rospy.loginfo('Starting data collection...')
    outcome = sm.execute()
