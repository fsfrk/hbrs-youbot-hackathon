#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <raw_msgs/BoundingBoxList.h>

class VisualizerNode
{

public:

  VisualizerNode(ros::NodeHandle& nh)
  {
    // Get topic names from the parameter server.
    ros::NodeHandle pn("~");
    std::string bounding_boxes_topic, bounding_boxes_visualization_topic;
    pn.param("bounding_boxes_topic", bounding_boxes_topic, std::string("/bounding_boxes"));
    pn.param("bounding_boxes_visualization_topic", bounding_boxes_visualization_topic, std::string("bounding_boxes_markers"));

    bounding_box_subscriber_ = nh.subscribe(bounding_boxes_topic, 5, &VisualizerNode::boundingBoxesCallback, this);
    marker_publisher_ = nh.advertise<visualization_msgs::Marker>(bounding_boxes_visualization_topic, 1);

    ROS_INFO("Started bounding box visualizer node.");
    ROS_INFO("Listening to \"%s\" topic.", bounding_boxes_topic.c_str());
    ROS_INFO("Publishing markers to \"%s\" topic.", bounding_boxes_visualization_topic.c_str());
  }

  void boundingBoxesCallback(const raw_msgs::BoundingBoxList::ConstPtr &list)
  {
    visualization_msgs::Marker lines;
    lines.header = list->header;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.scale.x = 0.001;
    lines.scale.y = 0.001;
    lines.color.a = 1.0;
    lines.ns = "bounding_boxes";
    lines.id = 1;
    lines.color.r = 1.0f;
    lines.color.g = 0.5f;
    lines.color.b = 0.5f;
    for (const auto& box : list->bounding_boxes)
    {
      const auto& pt = box.vertices;
      lines.points.push_back(pt[0]); lines.points.push_back(pt[1]);
      lines.points.push_back(pt[0]); lines.points.push_back(pt[3]);
      lines.points.push_back(pt[0]); lines.points.push_back(pt[4]);
      lines.points.push_back(pt[1]); lines.points.push_back(pt[2]);
      lines.points.push_back(pt[1]); lines.points.push_back(pt[5]);
      lines.points.push_back(pt[2]); lines.points.push_back(pt[3]);
      lines.points.push_back(pt[2]); lines.points.push_back(pt[6]);
      lines.points.push_back(pt[3]); lines.points.push_back(pt[7]);
      lines.points.push_back(pt[4]); lines.points.push_back(pt[5]);
      lines.points.push_back(pt[4]); lines.points.push_back(pt[7]);
      lines.points.push_back(pt[5]); lines.points.push_back(pt[6]);
      lines.points.push_back(pt[6]); lines.points.push_back(pt[7]);
    }
    marker_publisher_.publish(lines);
  }

private:

  ros::Subscriber bounding_box_subscriber_;
  ros::Publisher marker_publisher_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bounding_box_visualizer_node");
  ros::NodeHandle node;

  VisualizerNode vn(node);

  ros::spin();
  return 0;
}
