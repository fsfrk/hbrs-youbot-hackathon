#!/usr/bin/python
import roslib
roslib.load_manifest('raw_base_placement')
import rospy
import smach
import smach_ros
import raw_srvs.srv
import std_srvs.srv

        
# main
def main():
    rospy.init_node('raw_base_placement_test_script')

    #enable object perception
    object_finder_srv_start = rospy.ServiceProxy('/raw_perception/object_segmentation/start', std_srvs.srv.Empty)
    rospy.wait_for_service('/raw_perception/object_segmentation/start', 30)

    try:
        resp = object_finder_srv_start()
    except rospy.ServiceException, e:
        error_message = "%s"%e
        rospy.logerr("calling <</raw_perception/object_segmentation/start>> service not successfull, error: %s", error_message)
        return


    #getb object list
    object_finder_srv = rospy.ServiceProxy('/raw_perception/object_segmentation/get_segmented_objects', raw_srvs.srv.GetObjects)    
    rospy.wait_for_service('/raw_perception/object_segmentation/get_segmented_objects', 30)
    
    rospy.sleep(3)

    for i in range(20): 
        print "find object try: ", i
        resp = object_finder_srv()
          
        if (len(.objects) <= 0):
            rospy.loginfo('found no objects')
            rospy.slerespep(1);
        else:    
            rospy.loginfo('found {0} objects'.format(len(resp.objects)))
            break


    # BASE PLACEMENT
    base_placement_srv = rospy.ServiceProxy('/raw_base_placement/calculateOptimalBasePose', raw_srvs.srv.GetPoseStamped)    
    rospy.wait_for_service('/raw_base_placement/calculateOptimalBasePose', 30)

    for object in resp.objects:                   
        if object.pose.pose.position.z <= 0.05 and object.pose.pose.position.z >= 0.30:
            continue

        object_pose = object.pose
                       
        print "OBJ POSE: ", object_pose
 
        # call base placement service
        base_pose = base_placement_srv(object_pose)

        print "BASE_POSE", base_pose
   

    #stop object perception
    object_finder_srv_stop = rospy.ServiceProxy('/raw_perception/object_segmentation/stop', std_srvs.srv.Empty)
    rospy.wait_for_service('/raw_perception/object_segmentation/stop', 5)
    
    try:
        resp_stop = object_finder_srv_stop()
    except rospy.ServiceException, e:
        error_message = "%s"%e
        rospy.logerr("calling <</raw_perception/object_segmentation/stop>> service not successfull, error: %s", error_message)
        return 

if __name__ == '__main__':
    main()
