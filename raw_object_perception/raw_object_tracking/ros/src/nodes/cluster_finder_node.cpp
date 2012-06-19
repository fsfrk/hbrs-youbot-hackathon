#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <raw_srvs/GetDominantPlane.h>
#include <raw_srvs/GetClusters.h>
#include "object_tracker.h"
#include "occupancy_octree_object.h"
#include "tabletop_cluster_extractor.h"

class ClusterFinderNode
{

public:

  ClusterFinderNode(ros::NodeHandle& nh)
  {
    // Get service name from the parameter server.
    ros::NodeHandle pn("~");
    std::string find_clusters_service;
    pn.param("find_clusters_service", find_clusters_service, std::string("find_clusters"));
    find_service_ = nh.advertiseService(find_clusters_service, &ClusterFinderNode::findClustersCallback, this);
    ROS_INFO("Cluster finder service started.");
    // Create TabletopClusterExtractor
    tce_ = std::unique_ptr<TabletopClusterExtractor>(new TabletopClusterExtractor(0.009,  // point min height
                                                                                  1.000,  // point max height
                                                                                  0.010,  // object min height
                                                                                  0.100,  // object cluster tolerance
                                                                                  15,     // min cluster size
                                                                                  5000)); // max cluster size
    accumulation_duration_ = ros::Duration(10);
  }

  bool findClustersCallback(raw_srvs::GetClusters::Request& request, raw_srvs::GetClusters::Response& response)
  {
    ROS_INFO("Find clusters service requested.");
    // Get dominant plane to work with. This will block until server responds.
    if (!getDominantPlane()) return false;
    // Create new tracker.
    tracker_.reset(new ObjectTracker<OccupancyOctreeObject>);
    // Subscribe to the point clouds.
    ros::NodeHandle pn("~");
    std::string input_cloud_topic;
    pn.param("input_cloud_topic", input_cloud_topic, std::string("/camera/rgb/points"));
    ros::NodeHandle node;
    ros::Subscriber subscriber = node.subscribe(input_cloud_topic, 1, &ClusterFinderNode::cloudCallback, this);

    // Wait some time while data is being accumulated.
    ros::Time timeout = ros::Time::now() + ros::Duration(20);
    while (ros::Time::now() < timeout && ros::ok())
    {
      ros::spinOnce();
    }
    subscriber.shutdown();

    // Pack the response
    for (const auto& object : tracker_->getObjects())
    {
      PointCloud cloud;
      object->getPoints(cloud.points);
      cloud.header.frame_id = frame_id_;
      cloud.header.stamp = ros::Time::now();
      cloud.width = cloud.points.size();
      cloud.height = 1;
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(cloud, cloud_msg);
      response.clusters.push_back(cloud_msg);
    }
    return true;
  }

  /// Extract dominant plane using the external "GetDominantPlane" service.
  bool getDominantPlane()
  {
    // Get service name from the parameter server.
    ros::NodeHandle pn("~");
    std::string service_name;
    pn.param("extract_dominant_plane_service", service_name, std::string("extract_dominant_plane"));
    // Create a client for the service.
    ros::NodeHandle nh;
    ros::ServiceClient plane_extractor_client = nh.serviceClient<raw_srvs::GetDominantPlane>(service_name);
    // Request service and store the result in planar_polygon_ field.
    raw_srvs::GetDominantPlane srv;
    if (plane_extractor_client.call(srv))
    {
      PointCloud::VectorType contour;
      for (const auto& point : srv.response.contour)
      {
        PointT pt;
        pt.x = point.x;
        pt.y = point.y;
        pt.z = point.z;
        contour.push_back(pt);
      }
      Eigen::Vector4f coefficients(srv.response.coefficients.elems);
      planar_polygon_ = boost::make_shared<PlanarPolygon>(contour, coefficients);
      return true;
    }
    else
    {
      ROS_WARN("Call to dominant plane extractor failed.");
      return false;
    }
  }

  /// Cluster incoming point cloud and merge with existing clusters.
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud)
  {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*ros_cloud, *cloud);
    frame_id_ = ros_cloud->header.frame_id;

    if (!planar_polygon_) return;

    std::vector<PointCloud::Ptr> clusters;
    tce_->setInputCloud(cloud);
    tce_->setTablePolygon(planar_polygon_);
    tce_->extract(clusters);

    for (const PointCloud::Ptr& cluster : clusters)
      tracker_->addCluster(cluster);
  }

private:

  std::unique_ptr<TabletopClusterExtractor> tce_;
  std::unique_ptr<ObjectTracker<OccupancyOctreeObject>> tracker_;
  ros::ServiceServer find_service_;

  PlanarPolygonPtr planar_polygon_;

  std::string frame_id_;
  ros::Duration accumulation_duration_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluster_finder_node");
  ros::NodeHandle node;

  ClusterFinderNode tn(node);

  ros::spin();
  return 0;
}
