#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <raw_msgs/BoundingBox.h>
#include <raw_msgs/BoundingBoxList.h>
#include <raw_msgs/Object.h>
#include <raw_srvs/GetDominantPlane.h>
#include <raw_srvs/GetObjects.h>
#include <raw_srvs/RecognizeObject.h>
#include "bounding_box.h"
#include "object_tracker.h"
#include "occupancy_octree_object.h"
#include "tabletop_cluster_extractor.h"

class ObjectFinderNode
{

public:

  ObjectFinderNode(ros::NodeHandle& nh)
  {
    // Get service name from the parameter server.
    ros::NodeHandle pn("~");
    std::string find_objects_service, found_clusters_topic, bounding_boxes_topic;
    pn.param("find_objects_service", find_objects_service, std::string("find_objects"));
    pn.param("found_clusters_topic", found_clusters_topic, std::string("found_clusters"));
    pn.param("bounding_boxes_topic", bounding_boxes_topic, std::string("bounding_boxes"));
    find_service_ = nh.advertiseService(find_objects_service, &ObjectFinderNode::findObjectsCallback, this);
    clusters_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(found_clusters_topic, 1);
    bounding_boxes_publisher_ = nh.advertise<raw_msgs::BoundingBoxList>(bounding_boxes_topic, 1);
    object_labels_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("object_labels", 10);
    ROS_INFO("Object finder service started.");
    // Create TabletopClusterExtractor
    tce_ = std::unique_ptr<TabletopClusterExtractor>(new TabletopClusterExtractor(0.009,  // point min height
                                                                                  0.200,  // point max height
                                                                                  0.010,  // object min height
                                                                                  0.015,  // object cluster tolerance
                                                                                  20,     // min cluster size
                                                                                  5000)); // max cluster size

    srand(time(0));
    for (size_t i = 0; i < COLORS_NUM; ++i)
    {
      COLORS[i] = 1.0 * rand() / RAND_MAX;
    }
  }

  bool findObjectsCallback(raw_srvs::GetObjects::Request& request, raw_srvs::GetObjects::Response& response)
  {
    ROS_INFO("Find objects service requested.");
    // Get dominant plane to work with. This will block until server responds.
    if (!getDominantPlane()) return false;
    // Create new tracker.
    tracker_.reset(new ObjectTracker<OccupancyOctreeObject>);
    // Subscribe to the point clouds.
    ros::NodeHandle pn("~");
    std::string input_cloud_topic;
    pn.param("input_cloud_topic", input_cloud_topic, std::string("/camera/rgb/points"));
    ros::NodeHandle node;
    ros::Subscriber subscriber = node.subscribe(input_cloud_topic, 1, &ObjectFinderNode::cloudCallback, this);

    // Wait some time while data is being accumulated.
    int accumulation_timeout;
    pn.param("number_of_merges", merges_left_, 25);
    pn.param("accumulation_timeout", accumulation_timeout, 40);
    ros::Time timeout = ros::Time::now() + ros::Duration(accumulation_timeout);
    while (merges_left_ > 0 && ros::Time::now() < timeout && ros::ok())
    {
      ros::spinOnce();
    }
    subscriber.shutdown();

    // Pack the response
    ros::NodeHandle nh;
    //ros::ServiceClient object_recoginition_client = nh.serviceClient<raw_srvs::RecognizeObject>("recognize_object");

    response.stamp = ros::Time::now();
    size_t rejected = 0;
    for (const auto& object : tracker_->getObjects())
    {
      raw_msgs::Object object_msg;
      PointCloud cloud;
      object->getPoints(cloud.points);
      raw::BoundingBox box = raw::BoundingBox::create<PointT>(cloud.points, planar_polygon_->getCoefficients().head<3>());
      if (!isValid(cloud.points, box))
      {
        rejected++;
        continue;
      }
      cloud.header.frame_id = frame_id_;
      cloud.header.stamp = ros::Time::now();
      cloud.width = cloud.points.size();
      cloud.height = 1;
      pcl::toROSMsg(cloud, object_msg.cluster);
      geometry_msgs::Vector3 v;
      v.x = box.getLength();
      v.y = box.getWidth();
      v.z = box.getHeight();
      object_msg.dimensions.vector = v;
      object_msg.pose.header.frame_id = frame_id_;
/*
      // UGLY HACK
      raw_srvs::RecognizeObject srv;
      srv.request.points = cloud.points.size();
      srv.request.dimensions.vector = v;
      if (object_recoginition_client.call(srv))
      {
        object_msg.name = srv.response.name;
      }
      else
      {
        ROS_WARN("Call to object recognition service failed.");
        object_msg.name = "?";
      }
      // UGLY HACK
*/
      auto& pt = box.getCenter();
      geometry_msgs::Point center;
      center.x = pt[0];
      center.y = pt[1];
      center.z = pt[2];
      object_msg.pose.pose.position = center;
      response.objects.push_back(object_msg);
    }
    //publishObjectLabels(response);
    ROS_INFO("Rejected %zu object candidates.", rejected);
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
      planar_polygon_ = PlanarPolygonPtr(new PlanarPolygon(contour, coefficients));
      return true;
    }
    else
    {
      ROS_WARN("Call to dominant plane extractor failed.");
      return false;
    }
  }

  bool isValid(PointCloud::VectorType& points, raw::BoundingBox& box)
  {
    ros::NodeHandle pn("~");
    int filter_min_points;
    double filter_min_height;
    pn.param("filter_min_points", filter_min_points, 140);
    pn.param("filter_min_height", filter_min_height, 0.01);
    if (points.size() < (unsigned int)filter_min_points)
      return false;
    if (box.getHeight() < filter_min_height)
      return false;
    return true;
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

    ROS_INFO("Segmented %zu clusters in the new point cloud.", clusters.size());

    for (const PointCloud::Ptr& cluster : clusters)
      tracker_->addCluster(cluster);

    merges_left_--;
    ROS_INFO("Object tracker has %zu objects.", tracker_->getObjects().size());

    // Just in case someone is interested, publish clusters and bounding boxes..
    if (clusters_publisher_.getNumSubscribers())
      publishObjectClusters();
    if (bounding_boxes_publisher_.getNumSubscribers())
      publishBoundingBoxes(ros_cloud->header);
  }

  void publishObjectClusters()
  {
    pcl::PointCloud<pcl::PointXYZRGB> composite;
    size_t color = 0;
    for (const auto& object : tracker_->getObjects())
    {
      PointCloud::VectorType points;
      object->getPoints(points);
      color++;
      for (const auto& point : points)
      {
        pcl::PointXYZRGB pt;
        pt.x = point.x;
        pt.y = point.y;
        pt.z = point.z;
        pt.rgb = COLORS[color];
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
/*
  void publishObjectLabels(const raw_srvs::GetObjects::Response& response)
  {
    visualization_msgs::MarkerArray ma;
    int id = 1;
    for (const auto& object : response.objects)
    {
      visualization_msgs::Marker lines;
      lines.header.frame_id = "/openni_rgb_optical_frame";
      lines.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      lines.action = visualization_msgs::Marker::ADD;
      lines.scale.z = 0.04;
      lines.color.a = 1.0;
      lines.ns = "labels";
      lines.id = id++;
      lines.color.r = 1.0f;
      lines.color.g = 0.5f;
      lines.color.b = 0.5f;
      lines.pose = object.pose.pose;
      lines.pose.position.y += 0.02;
      lines.pose.orientation.w = 1.0;
      lines.text = object.name;
      ma.markers.push_back(lines);
    }
    object_labels_publisher_.publish(ma);
  }
*/
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

  std::unique_ptr<TabletopClusterExtractor> tce_;
  std::unique_ptr<ObjectTracker<OccupancyOctreeObject>> tracker_;
  ros::ServiceServer find_service_;
  ros::Publisher clusters_publisher_;
  ros::Publisher bounding_boxes_publisher_;
  ros::Publisher object_labels_publisher_;

  PlanarPolygonPtr planar_polygon_;

  std::string frame_id_;
  int merges_left_;

  static const size_t COLORS_NUM = 32;
  float COLORS[COLORS_NUM];

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluster_finder_node");
  ros::NodeHandle node;

  ObjectFinderNode tn(node);

  ros::spin();
  return 0;
}
