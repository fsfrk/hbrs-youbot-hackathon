#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <basescan_regression/OrientToBaseAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "BaseOrientation_Test");

  actionlib::SimpleActionClient<basescan_regression::OrientToBaseAction> ac("/scan_front_orientation", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  basescan_regression::OrientToBaseActionGoal goal;

  goal.goal.distance = 0.1;

  ac.sendGoal(goal.goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}



