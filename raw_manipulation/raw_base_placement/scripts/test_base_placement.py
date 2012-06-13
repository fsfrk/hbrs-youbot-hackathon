#!/usr/bin/python
import roslib
roslib.load_manifest('raw_base_placement')
import rospy
import geometry_msgs
import raw_srvs.srv
import std_srvs.srv
import tf

from simple_script_server import *
sss = simple_script_server()
        
# main
def main():
    rospy.init_node('raw_base_placement_test_script')

    ### tf listener
    tf_listener = tf.TransformListener()

    sss.move("arm", "flatposition")

    #enable object perception
    print "enable perception"
    object_finder_srv_start = rospy.ServiceProxy('/raw_perception/object_segmentation/start', std_srvs.srv.Empty)
    rospy.wait_for_service('/raw_perception/object_segmentation/start', 30)

    try:
        resp = object_finder_srv_start()
    except rospy.ServiceException, e:
        error_message = "%s"%e
        rospy.logerr("calling <</raw_perception/object_segmentation/start>> service not successfull, error: %s", error_message)
        return


    #getb object list
    print "get object list"
    object_finder_srv = rospy.ServiceProxy('/raw_perception/object_segmentation/get_segmented_objects', raw_srvs.srv.GetObjects)    
    rospy.wait_for_service('/raw_perception/object_segmentation/get_segmented_objects', 30)
        
        
    for i in range(20): 
        print "find object try: ", i
        resp = object_finder_srv()

        if (len(resp.objects) <= 0):
            rospy.loginfo('found no objects')
            rospy.sleep(1);
        else:
            rospy.loginfo('found {0} objects'.format(len(resp.objects)))
            break
    

    if (len(resp.objects) <= 0):
        rospy.loginfo('found NO objects')
        return

    # BASE PLACEMENT
    base_placement_srv = rospy.ServiceProxy('/raw_base_placement/calculateOptimalBasePose', raw_srvs.srv.GetPoseStamped) 
    print "wait for service: /raw_base_placement/calculateOptimalBasePose"   
    rospy.wait_for_service('/raw_base_placement/calculateOptimalBasePose', 30)


    for object_item in resp.objects:                   
    	if object_item.pose.pose.position.z <= 0.05 and object_item.pose.pose.position.z >= 0.30:
    	    continue
    
        object_pose = object_item.pose

        print "OBJ POSE: ", object_pose

        #transform base_link to map
        tf_listener.waitForTransform("/map", object_pose.header.frame_id, object_pose.header.stamp, rospy.Duration(10))
        obj_pose_transformed = tf_listener.transformPose("map", object_pose)

        print "OBJ POSE TRANSFORMED: ", obj_pose_transformed


        # call base placement service
        base_pose = base_placement_srv(obj_pose_transformed)

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


    ## DO MOVEMENT TO CALCULATED BASE POSE
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([base_pose.base_pose.pose.orientation.x, base_pose.base_pose.pose.orientation.y, base_pose.base_pose.pose.orientation.z, base_pose.base_pose.pose.orientation.w])
    sss.move("base", [base_pose.base_pose.pose.position.x, base_pose.base_pose.pose.position.y, yaw])

if __name__ == '__main__':
    main()
