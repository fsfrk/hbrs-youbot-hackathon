/*Transforms  x and y velocity to new x' and y' taking into 
consideration that the robot has to rotate  also while moving */

#ifndef RAW_VELOCITY_TRANSFORMATION_ROS_H_
#define RAW_VELOCITY_TRANSFORMATION_ROS_H_

#include <angles/angles.h>
#include <eigen/eigen.h>
#include <geometry_msgs/Twist.h>

// namespace raw_navigation
namespace raw_navigation {

	class VelocityTransformation
	{
		VelocityTransformation();
		SimpleVelocityTransformation(geometry_msgs::Twist& cmd_vel_);

	};

}
