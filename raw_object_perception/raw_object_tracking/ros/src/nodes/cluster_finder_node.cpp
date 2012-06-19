#include <string>
#include <vector>

#include <boost/format.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <pcl/filters/passthrough.h>

#include <raw_srvs/GetDominantPlane.h>
#include <raw_msgs/BoundingBox.h>
#include <raw_msgs/BoundingBoxList.h>
#include "bounding_box.h"
#include "object_tracker.h"
#include "occupancy_octree_object.h"
#include "organized_dominant_plane_extractor.h"
#include "tabletop_cluster_extractor.h"

typedef OccupancyOctreeObject ObjectT;

class ClusterFinderNode
{

public:

  ClusterFinderNode(ros::NodeHandle& nh)
  : tce_(new TabletopClusterExtractor)
  , tracker_(new ObjectTracker<ObjectT>)
  {
    // Get topic names from the parameter server.
    ros::NodeHandle pn("~");
    std::string input_cloud_topic, extract_dominant_plane_service, find_bounding_boxes_service;
    pn.param("input_cloud_topic", input_cloud_topic, std::string("/camera/rgb/points"));
    pn.param("extract_dominant_plane_service", extract_dominant_plane_service, std::string("extract_dominant_plane"));
    pn.param("find_object_candidates_service", find_object_candidates_service, std::string("find_object_candidates"));

    cloud_subscriber_ = nh.subscribe(input_cloud_topic, 5, &ClusterFinderNode::cloudCallback, this);
    plane_extractor_client_ = nh.serviceClient<raw_srvs::GetDominantPlane>(extract_dominant_plane_service);
    find_service_ = nh.advertiseService(find_object_candidates_service, &ClusterFinderNode::findCallback, this);
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud)
  {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*ros_cloud, *cloud);

    if (!planar_polygon_)
    {
      extractPlanarPolygon(cloud);
      frame_id_ = cloud->header.frame_id;
    }

    std::vector<PointCloud::Ptr> clusters;
    tce_->setInputCloud(cloud);
    tce_->setTablePolygon(planar_polygon_);
    tce_->extract(clusters);
    ROS_INFO("Number of clusters: %li.", clusters.size());

    for (const PointCloud::Ptr& cluster : clusters)
      tracker_->addCluster(cluster);

    publishObjectClusters();
    publishBoundingBoxes(ros_cloud->header);
  }

  bool findCallback(raw_srvs::GetD::Request& request, std_srvs::Empty::Response& response)
  {
    ROS_INFO("Tracker reset requested.");
    tracker_->reset();
    planar_polygon_.reset();
    return true;
  }

  bool saveCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    ROS_INFO("Save objects request.");
    boost::format fmt("object_%02i.pcd");
    int i = 0;
    for (const auto& object : tracker_->getObjects())
    {
      object->saveToPCD(boost::str(fmt % i++));
    }
    return true;
  }

  void extractPlanarPolygon(PointCloud::Ptr& cloud)
  {
    pcl::PassThrough<PointT> pass_through;
    pass_through.setFilterFieldName("z");
    double min_z_bound = 0.1, max_z_bound = 1.2;
    pass_through.setFilterLimits(min_z_bound, max_z_bound);
    pass_through.setKeepOrganized(true);
    pass_through.setInputCloud(cloud);
    PointCloud::Ptr cloud_filtered(new PointCloud);
    pass_through.filter(*cloud_filtered);
    planar_polygon_.reset(new PlanarPolygon);
    dpe_->setInputCloud(cloud_filtered);
    dpe_->setShrinkPlanePolygonRatio(0.07);
    dpe_->extract(*planar_polygon_);
    ROS_INFO_STREAM("Number of points in plane contour: " << planar_polygon_->getContour().size());
    ROS_INFO_STREAM("Plane coefficients:\n" << planar_polygon_->getCoefficients());
  }

  void publishObjectClusters()
  {
    PointCloud composite;
    size_t color = 0;
    for (const auto& object : tracker_->getObjects())
    {
      PointCloud::VectorType points;
      object->getPoints(points);
      color++;
      for (const auto& point : points)
      {
        PointT pt = point;
        pt.rgba = COLORS[color % COLORS_NUM];
        composite.points.push_back(pt);
      }
    }
    composite.header.frame_id = frame_id_;
    composite.width = composite.points.size();
    composite.height = 1;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(composite, cloud_msg);
    clusters_publisher_.publish(cloud_msg);
  }

  void publishBoundingBoxes(const std_msgs::Header& header)
  {
    raw_msgs::BoundingBoxList list_msg;
    list_msg.header = header;
    for (const auto& object : tracker_->getObjects())
    {
      PointCloud::VectorType points;
      object->getPoints(points);
      raw::BoundingBox box = raw::BoundingBox::create<PointT>(points, planar_polygon_->getCoefficients().head<3>());
      raw_msgs::BoundingBox box_msg;
      box_msg.length = box.getLength();
      box_msg.width = box.getWidth();
      box_msg.height = box.getHeight();
      for (const auto& vertex : box.getVertices())
      {
        geometry_msgs::Point pt;
        pt.x = vertex[0];
        pt.y = vertex[1];
        pt.z = vertex[2];
        box_msg.vertices.push_back(pt);
      }
      list_msg.bounding_boxes.push_back(box_msg);
    }
    bounding_boxes_publisher_.publish(list_msg);
  }

private:

  PlanarPolygonPtr planar_polygon_;

  std::unique_ptr<TabletopClusterExtractor> tce_;
  std::unique_ptr<ObjectTracker<ObjectT> > tracker_;
  ros::Subscriber cloud_subscriber_;
  ros::ServiceServer find_service_;
  ros::ServiceClient plane_extractor_client_;

  std::string frame_id_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_candidates_finder_node");
  ros::NodeHandle node;

  ClusterFinderNode tn(node);

  ros::spin();
  return 0;
}
