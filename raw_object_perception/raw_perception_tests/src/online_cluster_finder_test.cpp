#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>

#include <raw_srvs/GetClusters.h>
#include "aliases.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "online_cluster_finder_test");

  ros::Time::sleepUntil(ros::Time::now() + ros::Duration(5));

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<raw_srvs::GetClusters>("find_clusters");
  ros::Publisher clusters_publisher = nh.advertise<sensor_msgs::PointCloud2>("found_clusters", 1);
  raw_srvs::GetClusters srv;
  if (client.call(srv))
  {
    ROS_INFO("Number of clusters: %li", srv.response.clusters.size());
    if (!srv.response.clusters.size()) return 0;
    PointCloud composite;
    for (const auto& cluster : srv.response.clusters)
    {
      PointCloud::Ptr cloud(new PointCloud);
      pcl::fromROSMsg(cluster, *cloud);
      uint32_t color = rand() % 10000;
      for (const auto& point : cloud->points)
      {
        PointT pt = point;
        pt.rgba = color;
        composite.points.push_back(pt);
      }
    }
    composite.header = srv.response.clusters[0].header;
    composite.width = composite.points.size();
    composite.height = 1;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(composite, cloud_msg);
    clusters_publisher.publish(cloud_msg);
  }
  else
  {
    ROS_ERROR("Call to Find Clusters service failed.");
    return 1;
  }

  return 0;
}
