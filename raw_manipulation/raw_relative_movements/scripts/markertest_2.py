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

    goalframe = "/drawer_1"
    
    # call base placement service
    base_align = alignbase_srv(goalpose)  

    if base_align:
        rospy.loginfo("Action finished: %s", self.ac_base_adj.get_state())
        return 'succeeded'    
    else:
        rospy.logerr("Action did not finish before the time out!")
        return 'failed'  

if __name__ == '__main__':
    main()
