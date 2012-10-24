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
    ##shiftbase_srv = rospy.ServiceProxy('/raw_relative_movements/shiftbase', raw_srvs.srv.SetPoseStamped)
    alignbase_srv = rospy.ServiceProxy('/raw_relative_movements/alignwithmarker', raw_srvs.srv.SetMarkerFrame) 
    ##moveoptimalbase_srv = rospy.ServiceProxy('/raw_base_placement/moveoptimalbase', raw_srvs.srv.SetPoseStamped) 

    ##print "wait for service: /raw_relative_movements/shiftbase"   
    ##rospy.wait_for_service('/raw_relative_movements/shiftbase', 30)

    print "wait for service: /raw_relative_movements/alignwithmarker"   
    rospy.wait_for_service('/raw_relative_movements/alignwithmarker', 30)

    goalframe = "/base_link"
    
    # call base placement service
    base_align = alignbase_srv(goalframe)  

if __name__ == '__main__':
    main()
