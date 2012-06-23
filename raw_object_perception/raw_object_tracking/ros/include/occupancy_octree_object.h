#ifndef OCCUPANCY_OCTREE_OBJECT_H_
#define OCCUPANCY_OCTREE_OBJECT_H_

#include <string>

#include <pcl/octree/octree_pointcloud_occupancy.h>

#include "aliases.h"

class OccupancyOctreeObject
{

public:

    explicit OccupancyOctreeObject(const PointCloud::ConstPtr& cluster, double resolution = 0.0025);

    double distanceTo(const Eigen::Vector4f& point);

    void mergeCluster(const PointCloud::ConstPtr& cluster);

    void getPoints(PointCloud::VectorType& points) { octree_.getOccupiedVoxelCenters(points); }

    void saveToPCD(const std::string& filename);

protected:

    pcl::octree::OctreePointCloudOccupancy<PointT> octree_;

    int num_merges_;

};

#endif
