#!/usr/bin/python
#################################################################


import roslib
roslib.load_manifest('raw_base_placement_fred')

import rospy
import tf
import geometry_msgs.msg
import std_srvs.srv
import raw_srvs.srv
import hbrs_srvs.srv

from std_srvs.srv import *


class AdjustPoseWrtNearestObject:

    def __init__(self):
        # ToDo: add to launch file and read from param server
        self.distance_x = 0.15
        self.angle = 0.0
            
        #services
        self.srv_server_adjust_pose = rospy.Service('/raw_adjust_pose_wrt_nearest_obj/adjust_pose', std_srvs.srv.Empty, self.adjustPose)
        self.get_nearest_object = rospy.ServiceProxy('/hbrs_nearest_object_detector/GetNearestObject', hbrs_srvs.srv.GetPose)
        self.move_base_rel = rospy.ServiceProxy('/raw_relative_movements/shiftbase', raw_srvs.srv.SetPoseStamped)
    
    
    def adjustPose(self, req):
        
        
        # do the turn
        pose = 0
        try:
            rospy.wait_for_service('/hbrs_nearest_object_detector/GetNearestObject', 10)
            pose = self.get_nearest_object()
            
        except:
            print "service call <</hbrs_nearest_object_detector/GetNearestObject>> failed"     
        
        try:
            rospy.wait_for_service('/raw_relative_movements/shiftbase', 10)
            
            goalpose = geometry_msgs.msg.PoseStamped()
            goalpose.pose.position.z = 0.05     # abuse z to set descired velocity     
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            yaw = yaw - self.angle
            
            print "new yaw: ", yaw
            
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            goalpose.pose.orientation.x = quat[0]
            goalpose.pose.orientation.y = quat[1]
            goalpose.pose.orientation.z = quat[2]
            goalpose.pose.orientation.w = quat[3]
            
            self.move_base_rel(goalpose)
        except:
            print "service call <</raw_relative_movements/shiftbase>> failed"     
        
        
        print 'turn done'
        
        pose = 0
        try:
            rospy.wait_for_service('/hbrs_nearest_object_detector/GetNearestObject', 10)
            pose = self.get_nearest_object()
            
        except:
            print "service call <</hbrs_nearest_object_detector/GetNearestObject>> failed"     
        
        try:
            rospy.wait_for_service('/raw_relative_movements/shiftbase', 10)
            
            goalpose = geometry_msgs.msg.PoseStamped()
            goalpose.pose.position.x = pose.pose.position.x - self.distance_x
            print "DESIRED dist:", self.distance_x
            print "new x: ", goalpose.pose.position.x
            goalpose.pose.position.z = 0.05     # abuse z to set descired velocity

            quat = tf.transformations.quaternion_from_euler(0, 0, 0)
            goalpose.pose.orientation.x = quat[0]
            goalpose.pose.orientation.y = quat[1]
            goalpose.pose.orientation.z = quat[2]
            goalpose.pose.orientation.w = quat[3]
            
            self.move_base_rel(goalpose)
        except:
            print "service call <</raw_relative_movements/shiftbase>> failed"     
        

        return EmptyResponse()
        
        

if __name__ == '__main__':
    
    rospy.init_node('raw_adjust_pose_wrt_nearest_obj')
    
    adjust_pose_behavior = AdjustPoseWrtNearestObject()
    
    rospy.loginfo("adjust_pose_wrt_nearest_object behavior launched successfully")

    rospy.spin()
