//============================================================================
// Name        : Navigation module
// Author      : Praveen Ramanujam
// Version     :
// Copyright   : Your copyright notice
// Description : Class to find the robot pose delivered from amcl
//============================================================================

#include "raw_2dnav_iros2012/common.h"
#include "raw_2dnav_iros2012/robotpose.hpp"


geometry_msgs::Pose RobotPose::getRobotPose(double &yaw)
{
	  tf::StampedTransform transform;
	  try
	  {
	     listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
	     pose.position.x = transform.getOrigin().getX();
	     pose.position.y = transform.getOrigin().getY();
	     pose.orientation.x = transform.getRotation().x();
	     pose.orientation.y = transform.getRotation().y();
	     pose.orientation.z = transform.getRotation().z();
	     pose.orientation.w = transform.getRotation().w();

      }
	  catch (tf::TransformException ex)
	  {
	        //ROS_ERROR("%s",ex.what());
		    cout << "Waiting for transform.. Nothing in Cache" << endl;
	  }
	  tf::Quaternion q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
	  double pitch,roll;
	  btMatrix3x3(q).getEulerYPR(yaw,pitch,roll);
	  return pose;

}

