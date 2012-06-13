#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include "HomogenousTransform.h"
#include "KinematicSolver.h"
#include <raw_srvs/GetPoseStamped.h>

using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

bool calculateOptimalBasePose(raw_srvs::GetPoseStamped::Request  &req,
         raw_srvs::GetPoseStamped::Response &res )
{
  KinematicSolver youBot;
  // Solve IK
  Pose Goal;
  Goal << 500,500,0.5,0.5,0.5,0.5,0.5;
  JointParameter prefConfig(1,8);

  req.object_pose;

  res.base_pose;

  //youBot.solveIK(Goal,prefConfig);
  //res.sum = req.a + req.b;
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "kinematics_server");

  ros::NodeHandle n;

  //ros::ServiceServer service = n.advertiseService("calculateOptimalBasePose", calculateOptimalBasePose);  

  HomogenousTransform A = ht_from_xyzrpy(0.5,0.5,0.5,0.5,0.5,0.5);


  //cout << A.rotation().transpose();
  Pose Goal;
  KinematicSolver youBot;
  Goal << 0.4,0.3,0.5,0.5,0.5,0.5;
  JointParameter prefConfig(1,8);
  HomogenousTransform C = ht_from_eul(3.1416,-3.1416/2,0);
  //cout<< "The eul is " <<endl<<C.affine()<<endl;
  //Normalize(C);
  //cout<< "The norm eul is " <<endl<<C.affine()<<endl;
  prefConfig << 0.01,0.01,0.5,0.5,0.5,0.5,0.5,0.5;
  //cout << (youBot.calculateForwardKinematics(prefConfig)).affine()<<endl; 
  youBot.solveIK(Goal,prefConfig);
  cout << prefConfig;

  //ros::spin();

  ROS_INFO("Ready to estimate base position");
}
