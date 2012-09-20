#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "../srv_gen/cpp/include/raw_basic_navigation/RobotPose.h"


bool robotposeservice(raw_basic_navigation::RobotPose::Request  &req,
         raw_basic_navigation::RobotPose::Response &res )
{
	tf::StampedTransform transform;
	tf::TransformListener listener;
    try
    {
    	listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
    	res.pose.position.x = transform.getOrigin().getX();
        res.pose.position.y = transform.getOrigin().getY();
        res.pose.position.y = transform.getOrigin().getY();
        res.pose.orientation.x = transform.getRotation().x();
        res.pose.orientation.y = transform.getRotation().y();
        res.pose.orientation.z = transform.getRotation().z();
        res.pose.orientation.w = transform.getRotation().w();
    }
    catch (tf::TransformException ex)
    	  {
    	       //ROS_ERROR("%s",ex.what());
    	   	   std::cout << "Waiting for transform.. Nothing in Cache" << std::endl;
    	  }
	return true;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_pose_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("robot_pose", robotposeservice);
  ros::spin();

  return 0;
}
