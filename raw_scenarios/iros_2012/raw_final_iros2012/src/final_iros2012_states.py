#!/usr/bin/python

import roslib; roslib.load_manifest('raw_final_iros2012')
import rospy

import smach
from smach_ros import ServiceState

from geometry_msgs.msg import Vector3
from hbrs_srvs.srv import GetObjects
from raw_srvs.srv import PublishGoal, SetMarkerFrame


from simple_script_server import *
sss = simple_script_server()

DETECT_MARKERS = 'detect_markers'
PUBLISH_GOAL = 'publish_goal'
APPROACH_GOAL = '/raw_relative_movements/alignwithmarker'

class wait_for_task_marker(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_marker', 'missing_service'],
                                   output_keys=['task_marker_id'])
        try:
            rospy.wait_for_service(DETECT_MARKERS, timeout=5)
            self.detect_markers = rospy.ServiceProxy(DETECT_MARKERS, GetObjects)
        except rospy.ROSException:
            rospy.logwarn('Marker detection service is not available.')
            self.detect_markers = None

    def execute(self, userdata):
        if self.detect_markers is None:
            return 'missing_service'
        while True:
            response = self.detect_markers()
            if not response.objects:
                continue
            if len(response.objects) > 1:
                rospy.logwarn('Detected %i markers, will consider only the first one.' % len(response.objects))
            userdata.task_marker_id = response.objects[0].name
            return 'found_marker'

class adjust_pose_wrt_bin(smach.State):
    def __init__(self, distance=0.3):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'missing_service'],
                                   input_keys=['bin_marker_id'])
        self.displacement = Vector3()
        self.displacement.x = distance
        try:
            rospy.wait_for_service(PUBLISH_GOAL, timeout=5)
            self.publish_goal = rospy.ServiceProxy(PUBLISH_GOAL, PublishGoal)
        except rospy.ROSException:
            rospy.logwarn('[%s] service is not available.' % PUBLISH_GOAL)
            self.publish_goal = None
        try:
            rospy.wait_for_service(APPROACH_GOAL, timeout=5)
            self.approach_goal = rospy.ServiceProxy(APPROACH_GOAL, SetMarkerFrame)
        except rospy.ROSException:
            rospy.logwarn('[%s] service is not available.' % APPROACH_GOAL)
            self.approach_goal = None

    def execute(self, userdata):
        if self.publish_goal is None or self.approach_goal is None:
            return 'missing_service'
        self.publish_goal('/base_link', '/' + userdata.bin_marker_id, '/approach_bin_goal', self.displacement)
        if self.approach_goal('/approach_bin_goal'):
            return 'succeeded'
        else:
            return 'failed'

class grasp_bin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        return 'succeeded'

class pull_bin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        return 'succeeded'

class place_object_in_bin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        return 'succeeded'
