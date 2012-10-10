#!/usr/bin/python

import roslib; roslib.load_manifest('raw_final_iros2012')
import rospy

import smach
from smach_ros import ServiceState

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
            markers = self.detect_markers()
            if not markers:
                continue
            if len(markers) > 1:
                rospy.logwarn('Detected %i markers, will consider only the first one.' % len(markers))
            userdata.task_marker_id = markers[0].name
            return 'found_marker'

class adjust_pose_wrt_bin(smach.State, distance=0.3):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'missing_service'],
                                   input_keys=['bin_marker_id'])
        self.distance = distance
        try:
            rospy.wait_for_service(PUBLISH_GOAL, timeout=5)
            self.detect_markers = rospy.ServiceProxy(PUBLISH_GOAL, PublishGoal)
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
        self.publish_goal('/base_link', userdata.bin_marker_id, '/approach_bin_goal', distance, 0, 0)
        if self.approach_goal('/approach_bin_goal'):
            return 'succeeded'
        else:
            return 'failed'

class grasp_bin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'open_drawer_poses_not_available'])

    def execute(self, userdata):

          if (not rospy.has_param("/script_server/arm/open_drawer")):
            rospy.logerr("configuration for <<open_drawer>> NOT available on parameter server")
            return 'open_drawer_poses_not_available'
            
        pose_names = rospy.get_param("/script_server/arm/open_drawer")

        print pose_names
        
        grasp_poses = []
        for pose_name in pose_names:
            print "grasp pose: ", pose_name
            grasp_poses.append(("open_drawer/" + pose_name))
    
        grasp_poses.sort()

        print "sorted: ", grasp_poses

        return 'succeeded'
