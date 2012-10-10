#!/usr/bin/python

import roslib; roslib.load_manifest('raw_final_iros2012')
import rospy

import smach
from smach_ros import ServiceState

from geometry_msgs.msg import Vector3
from hbrs_srvs.srv import GetObjects, PassString
from raw_srvs.srv import PublishGoal, SetMarkerFrame

from simple_script_server import *
sss = simple_script_server()

DETECT_MARKERS = 'detect_markers'
PUBLISH_GOAL = 'publish_goal'
APPROACH_GOAL = '/raw_relative_movements/alignwithmarker'
FIND_WORKSPACE = 'find_workspace'
PLAY_SOUND = '/hbrs_audio/play_soundfile'

class do_nothing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self):
        return 'succeeded'


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
    def __init__(self, distance=0.5):
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
        try:
            self.approach_goal('/approach_bin_goal')
            return 'succeeded'
        except:
            return 'failed'

class grasp_bin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'open_drawer_poses_not_available'])

    def execute(self, userdata):

        sss.move("arm", "zeroposition")

        if (not rospy.has_param("/script_server/arm/open_drawer")):
            rospy.logerr("configuration for <<open_drawer>> NOT available on parameter server")
            return 'open_drawer_poses_not_available'
            
        pose_names = rospy.get_param("/script_server/arm/open_drawer")

        print pose_names
        
        grasp_poses = []
        for pose_name in pose_names:
            print "grasp pose: ", pose_name
            grasp_poses.append(("open_drawer/" + pose_name))
    
        #grasp_poses.sort()

        print "sorted: ", grasp_poses

        for pose_n in grasp_poses:
            raw_input("press enter")
            sss.move("arm", pose_n)

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

#class approach_pose_searching_for_box(smach.State):
    #def __init__(self):
        #smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             #input_keys=['base_pose_to_approach'])
        #try:
            #rospy.wait_for_service(FIND_WORKSPACE, timeout=5)
            #self.find_workspace = rospy.ServiceProxy(FIND_WORKSPACE, FindWorkspace)
        #except rospy.ROSException:
            #rospy.logwarn('Find workspace service is not available.')
            #self.find_workspace = None

    #def execute(self, userdata):
        ## upload proper constaraints
        #handle_base = sss.move('base', userdata.base_pose_to_approach)
        #while True:
            #response = self.find_workspace()
            #if response.polygon:

            #rospy.sleep(0.1)
            #base_state = handle_base.get_state()
            #if (base_state == actionlib.simple_action_client.GoalStatus.SUCCEEDED):
                #return "succeeded"
            #elif (base_state == actionlib.simple_action_client.GoalStatus.ACTIVE):
                #continue
            #else:
                #print 'last state: ',base_state
                #return "failed"

class point_and_announce_objects(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'missing_service'],
                                   input_keys=['recognized_objects'],
                                   output_keys=['recognized_objects'])
        try:
            rospy.wait_for_service(PLAY_SOUND, timeout=5)
            self.play_sound = rospy.ServiceProxy(PLAY_SOUND, PassString)
        except rospy.ROSException:
            rospy.logwarn('[%s] service is not available.' % PLAY_SOUND)
            self.play_sound  = None
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
       global planning_mode
       if planning_mode == '':
           sss.move('arm', 'zeroposition', mode=planning_mode)
       for obj in userdata.recognized_objects:
           try:
               p = self.get_pointing_position(obj)
           except RuntimeError, e:
               rospy.logerr('Skipping object: %s' % e)
               continue

            handle_arm = sss.move('arm', p, mode=planning_mode)
            # announce object name
            #srv_request = hbrs_srvs.srv.PassStringRequest()
            #request.str = object.name
            #try:
                #rospy.wait_for_service(self.play_sound_srv_name, 15)
                #resp = self.play_sound_srv(request)
            #except Exception, e:
                #rospy.logerr("could not execute service <<%s>>: %e", self.play_sound_srv_name, e)

            ## announce laying or standing
            #if(object.dimensions.vector.x > 0.04)
                #request.str = "standing.wav"
            #else
                #request.str = "laying.wav"
            #try:
                #rospy.wait_for_service(self.play_sound_srv_name, 15)
                #resp = self.play_sound_srv(request)
            #except Exception, e:
                #rospy.logerr("could not execute service <<%s>>: %e", self.play_sound_srv_name, e)
            sss.move('arm', 'zeroposition', mode=planning_mode)
        return 'succeeded'

    def get_pointing_position(self, obj):
        pose = self.transform_to_base_link(obj.pose)
        x = pose.pose.position.x + 0.1
        y = pose.pose.position.y - 0.005
        z = pose.pose.position.z + 0.1
        return [x, y, z, '/base_link']

    def transform_to_base_link(self, pose):
        for i in range(3):
            try:
                pose.header.stamp = rospy.Time.now()
                time = self.tf_listener.getLatestCommonTime('/base_link', pose.header.frame_id)
                return self.tf_listener.transformPose('/base_link', pose)
            except Exception, e:
                rospy.logerr("TF exception in point_and_announce_objects: %s", e)
                rospy.sleep(0.2)
        raise RuntimeError('Unable to transform object pose to base link.')
