#!/usr/bin/python
import roslib
roslib.load_manifest('raw_relative_movements')
import rospy
import geometry_msgs
import raw_srvs.srv
import std_srvs.srv
import tf

from simple_script_server import *
##sss = simple_script_server()
        
# main
def main():
    rospy.init_node('raw_base_placement_test_script')

    ### tf listener
    tf_listener = tf.TransformListener()

    ##sss.move("arm", "initposition")
    ##sss.move("arm", "pregrasp_front_init")
    
    # BASE PLACEMENT
    ##move_base_relative_srv = rospy.ServiceProxy('/raw_relative_movements/move_base_relative', raw_srvs.srv.SetPoseStamped)
    moveoptimalbase_srv = rospy.ServiceProxy('/raw_relative_movements/movetooptimalbase', raw_srvs.srv.SetPoseStamped) 
    ##moveoptimalbase_srv = rospy.ServiceProxy('/raw_base_placement/moveoptimalbase', raw_srvs.srv.SetPoseStamped) 

    ##print "wait for service: /raw_relative_movements/move_base_relative"   
    ##rospy.wait_for_service('/raw_relative_movements/move_base_relative', 30)

    print "wait for service: /raw_relative_movements/moveooptimalbase"   
    rospy.wait_for_service('/raw_relative_movements/movetooptimalbase', 30)

    goalpose = geometry_msgs.msg.PoseStamped()
    goalpose.pose.position.x = 0.1
    goalpose.pose.position.y = 0.1
    goalpose.pose.position.z = 0.1
    quat = tf.transformations.quaternion_from_euler(0,0,0)
    goalpose.pose.orientation.x = quat[0]
    goalpose.pose.orientation.y = quat[1]
    goalpose.pose.orientation.z = quat[2]
    goalpose.pose.orientation.w = quat[3]
    
    print "GOAL POSE TRANSFORMED: ", goalpose
    # call base placement service
    base_pose = moveoptimalbase_srv(goalpose)  

    goalpose = geometry_msgs.msg.PoseStamped()
    goalpose.pose.position.x = -0.1
    goalpose.pose.position.y = -0.1
    goalpose.pose.position.z = 0.1
    quat = tf.transformations.quaternion_from_euler(0,0,0)
    goalpose.pose.orientation.x = quat[0]
    goalpose.pose.orientation.y = quat[1]
    goalpose.pose.orientation.z = quat[2]
    goalpose.pose.orientation.w = quat[3]
    
    print "GOAL POSE TRANSFORMED: ", goalpose
    # call base placement service
    base_pose = moveoptimalbase_srv(goalpose) 

if __name__ == '__main__':
    main()
