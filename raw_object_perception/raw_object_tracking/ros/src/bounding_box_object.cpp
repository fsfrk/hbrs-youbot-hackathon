#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include "bounding_box_object.h"

BoundingBoxObject::BoundingBoxObject(const PointCloud::ConstPtr& cluster)
{
  pcl::getMinMax3D(*cluster, min_point_, max_point_);
  box_center_ = (max_point_ - min_point_) / 2.0 + min_point_;
}

double BoundingBoxObject::distanceTo(const Eigen::Vector4f& point) const
{
  Eigen::Vector4f box_center = (max_point_ - min_point_) / 2.0 + min_point_;
  return (box_center - point).norm();
}

void BoundingBoxObject::mergeCluster(const PointCloud::ConstPtr& cluster)
{
  Eigen::Vector4f min_point;
  Eigen::Vector4f max_point;
  pcl::getMinMax3D(*cluster, min_point, max_point);
  min_point_ = min_point_.cwiseMin(min_point);
  min_point_ = min_point_.cwiseMin(max_point);
  max_point_ = max_point_.cwiseMax(min_point);
  max_point_ = max_point_.cwiseMax(max_point);
  box_center_ = (max_point_ - min_point_) / 2.0 + min_point_;
}
