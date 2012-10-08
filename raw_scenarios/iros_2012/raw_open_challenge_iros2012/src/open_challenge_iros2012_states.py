#!/usr/bin/python
import roslib; roslib.load_manifest('raw_open_challenge_iros2012')
import rospy
import os
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

class point_to_and_announce_recognized_objects(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['recognized_objects'], output_keys=['recognized_objects'])
        
        self.play_sound_srv_name = "/hbrs_audio/play_soundfile"
        self.play_sound_srv = rospy.ServiceProxy(self.play_sound_srv_name, hbrs_srvs.srv.PassString)
        
        self.tf_listener = tf.TransformListener()
    def execute(self, userdata):
       
       global planning_mode
       
       if(planning_mode == ""):
           sss.move("arm", "zeroposition", mode=planning_mode)
 
       for object in userdata.recognized_objects:         

            # do transform
            tf_worked = False
            while not tf_worked:
                try:
                    print "HEADER: ", object.header.frame_id
                    object.header.stamp = rospy.Time.now()
                    self.tf_listener.waitForTransform('/base_link', object.header.frame_id, rospy.Time(0), rospy.Duration(5))
                    obj_pose_transformed = self.tf_listener.transformPose('/base_link', object)
                    tf_worked = True
                except Exception, e:
                    rospy.logerr("tf exception in place_base_in_front_of_object: transform: %s", e)
                    rospy.sleep(0.2)
                    tf_worked = False
            
            object = obj_pose_transformed

            #object.pose.pose.position.z = object.pose.pose.position.z + 0.02
            object.pose.pose.position.x = object.pose.pose.position.x + 0.1
            object.pose.pose.position.y = object.pose.pose.position.y - 0.005

            handle_arm = sss.move("arm", [object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z, "/base_link"], mode=planning_mode)
            
            # announce object name
            srv_request = hbrs_srvs.srv.PassStringRequest()
            request.str = object.name
            try:
                rospy.wait_for_service(self.play_sound_srv_name, 15)
                resp = self.play_sound_srv(request)
            except Exception, e:
                rospy.logerr("could not execute service <<%s>>: %e", self.play_sound_srv_name, e)

            # announce laying or standing
            if(object.dimensions.vector.x > 0.04)
                request.str = "standing.wav"
            else
                request.str = "laying.wav"
            try:
                rospy.wait_for_service(self.play_sound_srv_name, 15)
                resp = self.play_sound_srv(request)
            except Exception, e:
                rospy.logerr("could not execute service <<%s>>: %e", self.play_sound_srv_name, e)


            sss.move("arm", "zeroposition", mode=planning_mode)

        return 'succeeded'

