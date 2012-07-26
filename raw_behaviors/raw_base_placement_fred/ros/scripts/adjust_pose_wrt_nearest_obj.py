#!/usr/bin/python
#################################################################


import roslib
roslib.load_manifest('raw_base_placement_fred')

import rospy
import tf
import geometry_msgs.msg
import std_srvs.srv
import raw_srvs.srv
import brsu_srvs.srv

from std_srvs.srv import *


class AdjustPoseWrtNearestObject:

    def __init__(self):
        # ToDo: add to launch file and read from param server
        self.distance_x = 0.15
        self.angle = 0.0
        self.distance_x_threshold = 0.1
        self.angular_threshold = 0.01
        self.linear_speed = 0.05
        self.angular_speed = 0.05
        
        self.last_nearest_obj_msg = 0
        self.received_msg = False    
    
        # publisher / subscriber
        self.pub_base = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist)
        #self.sub_nearest_obj = rospy.Subscriber("/brsu_nearest_object_detector/nearest_object", geometry_msgs.msg.PoseStamped, self.cb_nearest_obj)
    
        #services
        self.srv_server_adjust_pose = rospy.Service('/raw_adjust_pose_wrt_nearest_obj/adjust_pose', std_srvs.srv.Empty, self.adjustPose)

        self.get_nearest_object = rospy.ServiceProxy('/brsu_nearest_object_detector/GetNearestObject', brsu_srvs.srv.GetPose)
        #self.nearest_object_start = rospy.ServiceProxy('/brsu_nearest_object_detector/start', Empty)
        #self.nearest_object_stop = rospy.ServiceProxy('/brsu_nearest_object_detector/stop', Empty)

        self.move_base_rel = rospy.ServiceProxy('/raw_relative_movements/shiftbase', raw_srvs.srv.SetPoseStamped)
    def cb_nearest_obj(self, msg):
        self.last_nearest_obj_msg = msg
        self.received_msg = True
        
    

    def adjustPose(self, req):
        
        pose = 0
        try:
            rospy.wait_for_service('/brsu_nearest_object_detector/start', 10)
            pose = self.get_nearest_object()
            
        except:
            print "service call <</brsu_nearest_object_detector/start>> failed"     
        
        try:
            rospy.wait_for_service('/raw_relative_movements/shiftbase', 10)
            
            goalpose = geometry_msgs.msg.PoseStamped()
            goalpose.pose.position.x = pose.pose.position.x - self.distance_x
            print "DESIRED dist:", self.distance_x
            print "new x: ", goalpose.pose.position.x
            goalpose.pose.position.y = 0.0
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
                
                
        return EmptyResponse()
        
        
        '''
        
        try:
            #start nearest obj calculation
            rospy.wait_for_service('/brsu_nearest_object_detector/start', 10)
            self.nearest_object_start()         
            
            target_reached = False
            angle_done = False
            
            while(not target_reached):
                # transform quaternion to euler to get the actual yaw to the object      
                
                self.received_msg = False
                while(not self.received_msg):
                    continue
                    
                
                  
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([self.last_nearest_obj_msg.pose.orientation.x, self.last_nearest_obj_msg.pose.orientation.y, self.last_nearest_obj_msg.pose.orientation.z, self.last_nearest_obj_msg.pose.orientation.w])
                dist_x = self.last_nearest_obj_msg.pose.position.x
                
                print "dist: %lf --- angle: %lf #### desired dist: %lf --- angle: %lf" % (dist_x, yaw, self.distance_x, self.angle)
                
                base_vel = geometry_msgs.msg.Twist()
                
                if not angle_done:
                    print "do angle"
                    if(yaw > (self.angle - self.angular_threshold) and yaw < (self.angle + self.angular_threshold)):
                        angle_done = True
                        print "########## ANGLE DONE"
                    elif(yaw < (self.angle + self.angular_threshold)):
                        base_vel.angular.z = -self.angular_speed
                    elif (yaw > (self.angle - self.angular_threshold)):   
                        base_vel.angular.z = self.angular_speed

                else:
                    if(dist_x > (self.distance_x - self.distance_x_threshold) and dist_x < (self.distance_x + self.distance_x_threshold)):
                        target_reached = True
                        print "####### OVERALL done"
                    elif(dist_x < (self.distance_x + self.distance_x_threshold)):
                        base_vel.linear.x = -self.linear_speed
                    elif (dist_x > (self.distance_x - self.distance_x_threshold)):
                        base_vel.linear.x = self.linear_speed
                    
                        
                
                self.pub_base.publish(base_vel)
            
            #stop calculation
            rospy.wait_for_service('/brsu_nearest_object_detector/stop', 10)
            self.nearest_object_stop()
            
    
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self.nearest_object_stop()
        
        return EmptyResponse()
    '''

if __name__ == '__main__':
    
    rospy.init_node('raw_adjust_pose_wrt_nearest_obj')
    
    adjust_pose_behavior = AdjustPoseWrtNearestObject()
    
    rospy.loginfo("adjust_pose_wrt_nearest_object behavior launched successfully")

    rospy.spin()
