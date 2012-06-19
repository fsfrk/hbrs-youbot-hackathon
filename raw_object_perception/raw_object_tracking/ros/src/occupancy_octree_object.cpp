#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_impl.h>

#include "occupancy_octree_object.h"

using namespace std;

OccupancyOctreeObject::OccupancyOctreeObject(const PointCloud::ConstPtr& cluster, double resolution)
: octree_(resolution)
, num_merges_(0)
{
  octree_.setInputCloud(cluster);
  octree_.addPointsFromInputCloud();
}

double OccupancyOctreeObject::distanceTo(const Eigen::Vector4f& point)
{
  PointCloud::VectorType points;
  octree_.getOccupiedVoxelCenters(points);
  double min_distance = std::numeric_limits<double>::max();
  for (const PointT& pt : points)
  {
    double distance = (pt.getVector4fMap() - point).norm();
    if (distance < min_distance)
    {
      min_distance = distance;
    }
  }
  return min_distance;
}

void OccupancyOctreeObject::mergeCluster(const PointCloud::ConstPtr& cluster)
{
  num_merges_++;
  for (const auto& point: cluster->points)
  {
    if ((point.x == point.x) && (point.y == point.y) && (point.z == point.z)) // NaN check
    {
      octree_.setOccupiedVoxelAtPoint(point);
    }
  }
}

void OccupancyOctreeObject::saveToPCD(const std::string& filename)
{
  PointCloud::Ptr pointcloud(new PointCloud);
  octree_.getOccupiedVoxelCenters(pointcloud->points);
  pointcloud->width = pointcloud->points.size();
  pointcloud->height = 1;
  pcl::io::savePCDFileASCII(filename, *pointcloud);
  PCL_INFO("Saved object to file: %s.\n", filename.c_str());
}
