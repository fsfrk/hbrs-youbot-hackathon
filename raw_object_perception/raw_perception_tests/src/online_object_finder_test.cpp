#include <boost/format.hpp>
#include <ros/ros.h>
#include <ros/console.h>

#include <raw_srvs/GetObjects.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "online_object_finder_test");
  ros::NodeHandle nh;

  ros::Time::sleepUntil(ros::Time::now() + ros::Duration(5));

  ros::ServiceClient client = nh.serviceClient<raw_srvs::GetObjects>("find_objects");
  raw_srvs::GetObjects srv;
  if (client.call(srv))
  {
    boost::format fmt("Object #%i: %2.1f,%2.1f,%2.1f,%6u : %s : %f");
    int cnt = 1;
    for (const auto& object : srv.response.objects)
    {
      double lenght = object.dimensions.vector.x * 100;
      double width = object.dimensions.vector.y * 100;
      double height = object.dimensions.vector.z * 100;
      double x = std::max(lenght, height);
      double y = std::min(lenght, height);
      double z = std::min(y, width);
      y = std::max(y, width);
      ROS_INFO_STREAM(fmt % cnt++ % x % y % z % object.cluster.width % object.name % object.pose.pose.orientation.x);
    }
  }
  else
  {
    ROS_ERROR("Call to Find Objects service failed.");
    return 1;
  }

  return 0;
}
