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
import numpy as np

from smach import State, StateMachine
from tf.transformations import quaternion_from_matrix
import tf

from label_visualizer import LabelVisualizer
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
        self.label_vis = LabelVisualizer('object_labels', 'teal')
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
        self.label_vis.publish(ud.names, [b.center for b in ud.bounding_boxes])
        return 'done'


class PackResponse(State):
    def __init__(self):
        State.__init__(self,
                       input_keys=['clusters',
                                   'bounding_boxes',
                                   'names',
                                   'response'],
                       output_keys=['response'],
                       outcomes=['done'])

    def execute(self, ud):
        ud.response = GetObjectsResponse()
        for c, b, n in zip(ud.clusters, ud.bounding_boxes, ud.names):
            obj = Object()
            q = self.get_orientation(b.vertices)
            obj.dimensions.vector = b.dimensions
            obj.pose.header = c.header
            obj.pose.pose.position = b.center
            obj.pose.pose.orientation.w = q[0]
            obj.pose.pose.orientation.x = q[1]
            obj.pose.pose.orientation.y = q[2]
            obj.pose.pose.orientation.z = q[3]
            obj.name = n
            ud.response.objects.append(obj)
        return 'done'

    def get_orientation(self, box):
        to_array = lambda msg: np.array([msg.x, msg.y, msg.z])
        n1 = to_array(box[4]) - to_array(box[0])
        n2 = to_array(box[1]) - to_array(box[0])
        n3 = to_array(box[3]) - to_array(box[0])
        n1_norm = np.linalg.norm(n1)
        n2_norm = np.linalg.norm(n2)
        n3_norm = np.linalg.norm(n3)
        n1 /= n1_norm
        n2 = n2 / n2_norm if n2_norm > n3_norm else n3 / n3_norm
        n3 = np.cross(n1, n2)
        m = np.array([[n1[0], n2[0], n3[0], 0],
                      [n1[1], n2[1], n3[1], 0],
                      [n1[2], n2[2], n3[2], 0],
                      [0,     0,     0,     1]])
        return quaternion_from_matrix(m)


class PublishTf(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['done'],
                       input_keys=['response'])
        self.tf = tf.TransformBroadcaster()

    def execute(self, ud):
        for i, obj in enumerate(ud.response.objects):
            p = obj.pose.pose.position
            q = obj.pose.pose.orientation
            self.tf.sendTransform((p.x, p.y, p.z), (q.w, q.x, q.y, q.z),
                                  rospy.Time.now(), 'object_%i' % i,
                                  obj.pose.header.frame_id)
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
                         PackResponse(),
                         transitions={'done': 'PUBLISH_TF'})
        StateMachine.add('PUBLISH_TF',
                         PublishTf(),
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
