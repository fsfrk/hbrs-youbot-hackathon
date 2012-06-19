#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/Marker.h>

#include "organized_dominant_plane_extractor.h"

class TestNode
{

public:

    TestNode(std::string input_cloud_topic)
    : dpe_(new OrganizedDominantPlaneExtractor)
    {
      ros::NodeHandle nh;
      cloud_subscriber_ = nh.subscribe(input_cloud_topic, 5, &TestNode::cloudCallback, this);
      marker_publisher_ = nh.advertise<visualization_msgs::Marker>("dominant_plane", 1);
      dpe_->setShrinkPlanePolygonRatio(0.08);
    }

protected:

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud)
    {
      PointCloud::Ptr cloud(new PointCloud);
      pcl::fromROSMsg(*ros_cloud, *cloud);

      pcl::PassThrough<PointT> pass_through;
      pass_through.setFilterFieldName("z");
      pass_through.setFilterLimits(0.7, 1.3);
      pass_through.setKeepOrganized(true);
      pass_through.setInputCloud(cloud);
      pass_through.filter(*cloud);

      PlanarPolygon planar_polygon;
      dpe_->setInputCloud(cloud);
      dpe_->extract(planar_polygon);
      ROS_INFO_STREAM("Number of points in plane contour: " << planar_polygon.getContour().size());
      ROS_INFO_STREAM("Plane coefficients:\n" << planar_polygon.getCoefficients());

      publishPlanarPolygon(planar_polygon, ros_cloud->header);
    }

    /** For the given PlanarPolygon draw a polyline through it points and also display the points themselves. */
    void publishPlanarPolygon(const PlanarPolygon& polygon, const std_msgs::Header& header)
    {
      visualization_msgs::Marker lines;
      lines.header = header;
      lines.type = visualization_msgs::Marker::LINE_LIST;
      lines.action = visualization_msgs::Marker::ADD;
      lines.scale.x = 0.005;
      lines.scale.y = 0.005;
      lines.color.a = 1.0;
      lines.ns = "planar_polygon";
      lines.id = 1;
      lines.color.r = 1.0f;
      lines.color.g = 0.5f;
      lines.color.b = 0.5f;

      const auto& contour = polygon.getContour();
      geometry_msgs::Point first_point;
      first_point.x = contour[0].x;
      first_point.y = contour[0].y;
      first_point.z = contour[0].z;
      lines.points.push_back(first_point);

      for (size_t i = 1; i < contour.size(); i++)
      {
        const auto& point = contour[i];
        geometry_msgs::Point pt;
        pt.x = point.x;
        pt.y = point.y;
        pt.z = point.z;
        lines.points.push_back(pt);
        lines.points.push_back(pt);
      }

      lines.points.push_back(first_point);
      marker_publisher_.publish(lines);
    }

    PointCloud::ConstPtr cloud_;
    std::unique_ptr<Eigen::Vector3f> plane_normal_;

private:

    std::unique_ptr<DominantPlaneExtractor> dpe_;
    ros::Subscriber cloud_subscriber_;
    ros::Publisher marker_publisher_;

};


int main (int argc, char** argv)
{
  ros::init(argc, argv, "online_dominant_plane_extractor_test");

  if (argc != 2)
  {
    ROS_ERROR("Usage: %s <cloud topic>", argv[0]);
    return 1;
  }

  TestNode tn(argv[1]);

  ros::spin();
  return 0;
}
