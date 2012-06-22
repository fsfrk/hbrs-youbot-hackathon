#!/usr/bin/python
import roslib
roslib.load_manifest('raw_base_placement')
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
    base_placement_srv = rospy.ServiceProxy('/raw_base_placement/calculateOptimalBasePose', raw_srvs.srv.GetPoseStamped) 
    print "wait for service: /raw_base_placement/calculateOptimalBasePose"   
    rospy.wait_for_service('/raw_base_placement/calculateOptimalBasePose', 30)

    obj_pose = geometry_msgs.msg.PoseStamped()
    obj_pose.pose.position.x = 1.7
    obj_pose.pose.position.y = 0.5
    obj_pose.pose.position.z = 0.2
    quat = tf.transformations.quaternion_from_euler(0,0,0)
    obj_pose.pose.orientation.x = quat[0]
    obj_pose.pose.orientation.y = quat[1]
    obj_pose.pose.orientation.z = quat[2]
    obj_pose.pose.orientation.w = quat[3]
    
    print "OBJ POSE TRANSFORMED: ", obj_pose
    # call base placement service
    base_pose = base_placement_srv(obj_pose)

    print "BASE_POSE", base_pose
  

    ## DO MOVEMENT TO CALCULATED BASE POSE
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([base_pose.base_pose.pose.orientation.x, base_pose.base_pose.pose.orientation.y, base_pose.base_pose.pose.orientation.z, base_pose.base_pose.pose.orientation.w])
    ##sss.move("base", [base_pose.base_pose.pose.position.x, base_pose.base_pose.pose.position.y, yaw])
    ##sss.move("base", [1.24,0,0])


if __name__ == '__main__':
    main()
