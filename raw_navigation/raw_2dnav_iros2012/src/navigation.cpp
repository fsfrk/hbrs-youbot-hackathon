//============================================================================
// Name        : Navigation module
// Author      : Praveen Ramanujam
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

#include <iostream>
#include "raw_2dnav_iros2012/common.h"
#include "raw_2dnav_iros2012/robotpose.hpp"


int main(int argc, char** argv) {

	ros::init(argc, argv, "tf_");
	ros::NodeHandle node;
	double yaw;
	tf::TransformListener tf(ros::Duration(10));
	RobotController pose;
	geometry_msgs::Pose robotpose;
	ros::Rate rate(10.0);
	while (ros::ok())
	{
		//Get Current Robot Pose
	     robotpose = pose.getRobotPose(yaw);
	     cout << "[" << robotpose.position.x <<"  "<< robotpose.position.y <<"  "<<yaw<<"]" << endl;
	     rate.sleep();
	}

	return 0;
}

