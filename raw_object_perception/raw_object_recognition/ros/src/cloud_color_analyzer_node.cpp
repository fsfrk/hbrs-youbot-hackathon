#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <raw_srvs/AnalyzeCloudColor.h>

bool analyzeCallback(raw_srvs::AnalyzeCloudColor::Request& request, raw_srvs::AnalyzeCloudColor::Response& response)
{
  typedef boost::accumulators::tag::mean mean;
  typedef boost::accumulators::tag::median median;
  boost::accumulators::accumulator_set<float, boost::accumulators::stats<mean, median>> color;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(request.cloud, cloud);
  for (const auto& point : cloud.points)
    color(0.2989f * point.r + 0.5870f * point.g + 0.1140f * point.b);
  response.mean = boost::accumulators::mean(color);
  response.median = boost::accumulators::median(color);
  response.points = cloud.points.size();
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_color_analyzer");
  ros::NodeHandle nh;
  ros::ServiceServer analyzer_service = nh.advertiseService("analyze_cloud_color", analyzeCallback);
  ros::spin();
  return 0;
}

