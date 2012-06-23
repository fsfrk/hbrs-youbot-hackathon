#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_impl.h>

#include "colored_occupancy_octree_object.h"

using namespace std;

ColoredOccupancyOctreeObject::ColoredOccupancyOctreeObject(const PointCloud::ConstPtr& cluster, double resolution)
: OccupancyOctreeObject(cluster, resolution)
, color_(0.0)
{
}

void ColoredOccupancyOctreeObject::mergeCluster(const PointCloud::ConstPtr& cluster)
{
  num_merges_++;
  for (const auto& point: cluster->points)
  {
    if ((point.x == point.x) && (point.y == point.y) && (point.z == point.z)) // NaN check
    {
      octree_.setOccupiedVoxelAtPoint(point);
      color_(1.0 * point.rgba / 0xFFFFFFFF);
    }
  }
}

