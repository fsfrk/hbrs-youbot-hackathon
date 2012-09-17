/*Checks the feasibility of the goal. 
  In other words, pre-calculates if there will be
  problem after reaching the goal. */


#include "ros/ros.h"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <../srv_gen/cpp/include/raw_2dnav_iros/PoseFeasibility.h>

using namespace std;

namespace movebase
{

}
bool check_feasibility(raw_2dnav_iros::PoseFeasibility::Request  &goal,
         raw_2dnav_iros::PoseFeasibility::Response &feasible )
{



  ROS_INFO("sending back response");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_feasibility");
  ros::NodeHandle n;
  /*Initialize costmap and check if the goal is near the goal position*/
  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS costmap("global_costmap", tf);
  costmap.pause();
  costmap.start();
  // costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", tf);

 while (1)
 {
    /*Get the inflation radius, circumcenter and incenter of the robot*/
    double inflation = costmap.getInflationRadius();
    double incenter  = costmap.getInscribedRadius();
    double circumcenter = costmap.getCircumscribedRadius();
    double resolution = costmap.getResolution();


    cout << "Incenter" << incenter << endl << "circumcenter" << resolution << endl;

  //ros::ServiceServer service = n.advertiseService("goal_feasible", check_feasibility);
    ros::spin();
 }
  return 0;
}
