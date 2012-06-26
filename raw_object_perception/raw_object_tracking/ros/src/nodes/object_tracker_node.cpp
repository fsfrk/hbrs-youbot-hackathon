#include <string>
#include <vector>

#include <boost/format.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <pcl/filters/passthrough.h>

#include <raw_msgs/BoundingBox.h>
#include <raw_msgs/BoundingBoxList.h>
#include "bounding_box.h"
#include "object_tracker.h"
#include "occupancy_octree_object.h"
#include "organized_dominant_plane_extractor.h"
#include "tabletop_cluster_extractor.h"

typedef OccupancyOctreeObject ObjectT;

class TrackerNode
{

public:

  TrackerNode(ros::NodeHandle& nh)
  : dpe_(new OrganizedDominantPlaneExtractor)
  , tce_(new TabletopClusterExtractor)
  , tracker_(new ObjectTracker<ObjectT>)
  {
    // Get topic names from the parameter server.
    ros::NodeHandle pn("~");
    std::string input_cloud_topic, output_clusters_topic, bounding_boxes_topic;
    pn.param("input_cloud_topic", input_cloud_topic, std::string("/camera/rgb/points"));
    pn.param("output_clusters_topic", output_clusters_topic, std::string("clusters"));
    pn.param("bounding_boxes_topic", bounding_boxes_topic, std::string("bounding_boxes"));

    cloud_subscriber_ = nh.subscribe(input_cloud_topic, 5, &TrackerNode::cloudCallback, this);
    bounding_boxes_publisher_ = nh.advertise<raw_msgs::BoundingBoxList>(bounding_boxes_topic, 1);
    clusters_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(output_clusters_topic, 1);
    reset_service_ = nh.advertiseService("reset", &TrackerNode::resetCallback, this);
    save_service_ = nh.advertiseService("save", &TrackerNode::saveCallback, this);

    for (size_t i = 0; i < COLORS_NUM; ++i)
    {
      COLORS[i] = rand() % 10000;
    }
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
    ROS_INFO_STREAM("Number of clusters: " << clusters.size());

    for (const PointCloud::Ptr& cluster : clusters)
      tracker_->addCluster(cluster);

    publishObjectClusters();
    publishBoundingBoxes(ros_cloud->header);
  }

  bool resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
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
    dpe_->setShrinkPlanePolygonBy(0.03);
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

  std::unique_ptr<DominantPlaneExtractor> dpe_;
  std::unique_ptr<TabletopClusterExtractor> tce_;
  std::unique_ptr<ObjectTracker<ObjectT> > tracker_;
  ros::Subscriber cloud_subscriber_;
  ros::Publisher bounding_boxes_publisher_;
  ros::Publisher clusters_publisher_;
  ros::ServiceServer reset_service_;
  ros::ServiceServer save_service_;

  std::string frame_id_;

  static const size_t COLORS_NUM = 32;
  uint32_t COLORS[COLORS_NUM];

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracker_node");
  ros::NodeHandle node;

  TrackerNode tn(node);

  ros::spin();
  return 0;
}
