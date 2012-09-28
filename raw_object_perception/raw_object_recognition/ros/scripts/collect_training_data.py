#!/usr/bin/env python

PACKAGE = 'raw_object_recognition'
NODE = 'collect_training_data'

import roslib
roslib.load_manifest(PACKAGE)
import sys
from os.path import join

# Import helper states for user input and counting
sys.path.append(join(roslib.packages.get_pkg_dir(PACKAGE), 'common', 'src'))
from confirm_state import ConfirmState
from counter_state import CounterState

# Import states for calling services from hbrs_scene_segmentation
sys.path.append(join(roslib.packages.get_pkg_dir('raw_object_detection'),
                     'ros', 'src'))
import service_states

import rospy
import argparse

from smach import State, StateMachine, cb_interface
from smach_ros import ServiceState, ConditionState

from raw_srvs.srv import AnalyzeCloudColor, AnalyzeCloudColorRequest

from dataset import Dataset


class StoreObject(State):
    def __init__(self, dataset, object_id):
        State.__init__(self,
                       outcomes=['stored'],
                       input_keys=['bounding_boxes', 'clusters', 'mean',
                                   'median', 'points'])
        base = roslib.packages.get_pkg_dir(PACKAGE)
        self.dataset = Dataset(join(base, 'common', 'data'), dataset)
        self.object_id = object_id

    def execute(self, ud):
        self.dataset.store(self.object_id, ud.bounding_boxes[0].dimensions,
                           ud.points, ud.mean, ud.median)
        return 'stored'


if __name__ == '__main__':
    rospy.init_node(NODE)
    parser = argparse.ArgumentParser(description='''
    Collect training data for object recognition.
    To store sample press Enter, to drop press any other key.
    ''')
    parser.add_argument('object_id', help='id of the object (as in database)')
    parser.add_argument('--dataset', help='dataset name (default "standard")',
                        default='standard')
    parser.add_argument('--confirm-every', help=('ask for confirmation after '
                        'this many successful calls to the object detection '
                        'service'), default=1)
    args = parser.parse_args()
    sm = StateMachine(['succeeded', 'aborted', 'preempted'])
    with sm:
        StateMachine.add('FIND_WORKSPACE',
                         service_states.find_workspace,
                         transitions={'succeeded': 'CONFIRM_WORKSPACE',
                                      'aborted': 'FIND_WORKSPACE'})
        StateMachine.add('CONFIRM_WORKSPACE',
                         ConfirmState('Is the detected workspace correct?'),
                         transitions={'yes': 'ACCUMULATE_CLOUD',
                                      'no': 'FIND_WORKSPACE'})
        StateMachine.add('ACCUMULATE_CLOUD',
                         service_states.accumulate_tabletop_cloud,
                         transitions={'succeeded': 'CLUSTER_CLOUD'})
        StateMachine.add('CLUSTER_CLOUD',
                         service_states.cluster_tabletop_cloud,
                         transitions={'succeeded': 'IS_ONLY_ONE_OBJECT'})
        StateMachine.add('IS_ONLY_ONE_OBJECT',
                         ConditionState(cond_cb=lambda userdata:
                                        len(userdata.clusters) == 1,
                                        input_keys=['clusters']),
                         transitions={'true': 'MAKE_BOUNDING_BOXES',
                                      'false': 'ACCUMULATE_CLOUD'})
        StateMachine.add('MAKE_BOUNDING_BOXES',
                         service_states.make_bounding_boxes,
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
                         CounterState(int(args.confirm_every)),
                         transitions={'overflow': 'CONFIRM_OBJECT',
                                      'counting': 'STORE_OBJECT'})
        StateMachine.add('CONFIRM_OBJECT',
                         ConfirmState('Is the detected object correct?'),
                         transitions={'yes': 'STORE_OBJECT',
                                      'no': 'ACCUMULATE_CLOUD'})
        StateMachine.add('STORE_OBJECT',
                         StoreObject(args.dataset, args.object_id),
                         transitions={'stored': 'ACCUMULATE_CLOUD'})
    rospy.loginfo('Starting data collection...')
    outcome = sm.execute()
