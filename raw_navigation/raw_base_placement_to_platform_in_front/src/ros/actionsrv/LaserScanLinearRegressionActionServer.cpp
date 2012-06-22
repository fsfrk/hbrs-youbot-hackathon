#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>


#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include "../../LaserScanLinearRegression.h"
#include <raw_base_placement_to_platform_in_front/OrientToBaseAction.h>
#include <raw_base_placement_to_platform_in_front/BaseScanLinearRegression.h>
#include <iostream>

using namespace raw_base_placement_to_platform_in_front;



class OrientToLaserReadingAction {
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<OrientToBaseAction> as_;
	std::string action_name_;

	std::string service_name;
	std::string cmd_vel_topic;

	float target_distance;
	float max_velocity;

        ros::Publisher cmd_pub;
        ros::ServiceClient client;

public:


	OrientToLaserReadingAction(ros::NodeHandle nh, std::string name, std::string cmd_vel_topic, std::string linreg_service_name) :
		as_(nh,	name, boost::bind(&OrientToLaserReadingAction::executeActionCB, this, _1), false)
	{
		//as_.registerGoalCallback();
		this->action_name_ = name;
		this->service_name = linreg_service_name;
		this->cmd_vel_topic = cmd_vel_topic;

		nh_ = nh;

		target_distance = 0.07;
		max_velocity = 0.1;

		
		ROS_INFO("Register publisher");

		cmd_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000);

		ROS_INFO("Create service clinet");

		client = nh_.serviceClient<BaseScanLinearRegression>(service_name);

		as_.start();

	}



	geometry_msgs::Twist calculateVelocityCommand(double center, double a, double b) {
		geometry_msgs::Twist cmd;

		std::cout << "HHHHH: " << fabs(b) << std::endl;
		if (fabs(b) > 0.5) 
		{
		  cmd.angular.z = -b;
			//cmd_pub.publish(cmd);
			//std::cout << "cmd.angular.z:  " << cmd.angular.z << std::endl;

		}

		/*
		else if (fabs(center) > 0.005) 
		{
			cmd.linear.y = center / 3;
			//cmd_pub.publish(cmd);
			//std::cout << "cmd.linear.y:  " << cmd.linear.y << std::endl;
		}*/	
		
		else if (a > target_distance) 
		{
			cmd.linear.x = a / 3;
			//std::cout << "cmd.linear.x:  " << cmd.linear.x << std::endl;
			//cmd_pub.publish(cmd);

		}

		if (cmd.linear.x > max_velocity) 
			cmd.linear.x = max_velocity;
		else if (cmd.linear.x < -max_velocity) 
			cmd.linear.x = -max_velocity;

		if (cmd.linear.y > max_velocity) 
			cmd.linear.y = max_velocity;
		else if (cmd.linear.y < -max_velocity) 
			cmd.linear.y = -max_velocity;

		if (cmd.angular.z > max_velocity)
			cmd.angular.z = max_velocity;
		else if (cmd.angular.z < -max_velocity)
			cmd.angular.z = -max_velocity;

		return cmd;
	}


	void executeActionCB(const OrientToBaseGoalConstPtr& goal) {

		BaseScanLinearRegression srv;

		srv.request.filter_minAngle = -M_PI_4;
		srv.request.filter_maxAngle = M_PI_4;
		srv.request.filter_minDistance = 0.05;
		srv.request.filter_maxDistance = 0.50;

		target_distance = goal->distance;

		ros::Duration max_time(30.0);
		ros::Time stamp = ros::Time::now();
		OrientToBaseResult result;

		while (true) {
			ROS_INFO("Call service Client");

			if(client.call(srv)) {
				ROS_INFO("Called service LaserScanLinearRegressionService");
				//std::cout << "result: " << srv.response.center << ", " << srv.response.a << ", " << srv.response.b << std::endl;

				geometry_msgs::Twist cmd = calculateVelocityCommand(srv.response.center, srv.response.a, srv.response.b);
				cmd_pub.publish(cmd);

				std::cout << "cmd x:" << cmd.linear.x << ", y: "  << cmd.linear.y << ", z: " << cmd.angular.z << std::endl;

				if (fabs(cmd.linear.x) + fabs(cmd.angular.z) < 0.00001) {
			    
					ROS_INFO("Point reached");
					result.succeed = true;
					as_.setSucceeded(result);
					break;
				}

				if  (stamp + max_time < ros::Time::now()) {
					result.succeed = false;
				        as_.setAborted(result);
				    	break;
				}

			} else {
				ROS_ERROR("Failed to call service LaserScanLinearRegressionService");

				if  (stamp + max_time < ros::Time::now()) {
					result.succeed = false;
					as_.setAborted(result);
					break;
				}
			}
		}

	}
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "OrientToLaserReading");
  ros::NodeHandle n;

  std::string cmd_vel_name = "/cmd_vel";
  std::string service_name = "/scan_front_linearregression";

  OrientToLaserReadingAction orientAction(n, "/scan_front_orientation", cmd_vel_name, service_name);

  ROS_INFO("Action Service is ready");

  while (ros::ok()) {
     ros::spinOnce();
  }
  return 0;
}

