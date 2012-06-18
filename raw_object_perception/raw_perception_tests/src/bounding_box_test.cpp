#include <ctime>
#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "bounding_box.h"
#include "test_base.hpp"

using namespace raw;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class BoundingBoxTest : public TestBase
{

public:

    BoundingBoxTest(PointCloud::ConstPtr cloud, const Eigen::Vector3f& normal)
    : TestBase()
    , cloud_(cloud)
    , normal_(normal)
    { }

protected:

    virtual void process()
    {
      BoundingBox box = BoundingBox::create<PointT>(cloud_, normal_);

      viewer_.removeAllPointClouds(0);
      viewer_.removeAllShapes(0);

      pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_, 0, 255, 0);
      viewer_.addPointCloud<PointT>(cloud_, single_color, "cloud");

      displayNormal(normal_);
      displayBoundingBox(box);
    }

    void displayBoundingBox(const BoundingBox box)
    {
      BoundingBox::Points vertices = box.getVertices();
      PointCloud::Ptr bottom(new PointCloud);
      PointCloud::Ptr top(new PointCloud);
      for (size_t i = 0; i < 4; i++)
      {
        PointT pt;
        pt.getArray3fMap() = vertices[i];
        bottom->points.push_back(pt);
        pt.getArray3fMap() = vertices[4 + i];
        top->points.push_back(pt);
      }
      viewer_.addPolygon<PointT>(bottom, 255, 0, 0, "bottom_poly");
      viewer_.addPolygon<PointT>(top, 255, 0, 0, "top_poly");
      viewer_.addLine(bottom->points[0], top->points[0], 255, 0, 0, "vertical1");
      viewer_.addLine(bottom->points[1], top->points[1], 255, 0, 0, "vertical2");
      viewer_.addLine(bottom->points[2], top->points[2], 255, 0, 0, "vertical3");
      viewer_.addLine(bottom->points[3], top->points[3], 255, 0, 0, "vertical4");
    }

    PointCloud::ConstPtr cloud_;
    Eigen::Vector3f normal_;

};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "bounding_box_test");

  if (argc != 2 && argc != 5)
  {
    PCL_ERROR("Usage: %s <filename.pcd> [A B C]\n", argv[0]);
    return 1;
  }

  srand(time(0));

  PointCloud::Ptr cloud(new PointCloud);
  pcl::io::loadPCDFile(argv[1], *cloud);

  Eigen::Vector3f normal;
  if (argc == 5)
  // Plane coefficients supplied, parse them into vector.
    for (size_t i = 0; i < 3; i++)
      normal[i] = boost::lexical_cast<float>(argv[i + 2]);
  else // Generate random normal.
    for (size_t i = 0; i < 3; i++)
      normal[i] = 1.0 * rand() / RAND_MAX;

  std::cout << "Plane normal:\n" << normal << std::endl;

  BoundingBoxTest bbt(cloud, normal);
  bbt.run();

  return 0;
}
