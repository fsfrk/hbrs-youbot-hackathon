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
    shiftbase_srv = rospy.ServiceProxy('/raw_relative_movements/shiftbase', raw_srvs.srv.SetPoseStamped) 
    ##moveoptimalbase_srv = rospy.ServiceProxy('/raw_base_placement_nirmal/moveoptimalbase', raw_srvs.srv.SetPoseStamped) 

    print "wait for service: /raw_relative_movements/shiftbase"   
    rospy.wait_for_service('/raw_relative_movements/shiftbase', 30)

    goalpose = geometry_msgs.msg.PoseStamped()
    goalpose.pose.position.x = -0.3
    goalpose.pose.position.y = -0.1
    goalpose.pose.position.z = 0.0
    quat = tf.transformations.quaternion_from_euler(0,0,0)
    goalpose.pose.orientation.x = quat[0]
    goalpose.pose.orientation.y = quat[1]
    goalpose.pose.orientation.z = quat[2]
    goalpose.pose.orientation.w = quat[3]
    
    print "GOAL POSE TRANSFORMED: ", goalpose
    # call base placement service
    base_pose = shiftbase_srv(goalpose)  


if __name__ == '__main__':
    main()
