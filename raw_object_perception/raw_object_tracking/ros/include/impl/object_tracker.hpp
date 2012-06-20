#ifndef OBJECT_TRACKER_HPP_
#define OBJECT_TRACKER_HPP_

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

template<typename ObjectT>
ObjectTracker<ObjectT>::ObjectTracker()
: merge_threshold_(0.05)
{
}

template<typename ObjectT>
ObjectTracker<ObjectT>::~ObjectTracker()
{
}

template<typename ObjectT>
void ObjectTracker<ObjectT>::addCluster(const PointCloud::ConstPtr& cluster)
{
  PCL_INFO("[ObjectTracker::addCluster] Got a cluster with %6li points. ", cluster->points.size());
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);
  double min_distance = std::numeric_limits<double>::max();
  size_t min_distance_id = 0;
  for (size_t i = 0; i < objects_.size(); i++)
  {
    double distance = objects_[i]->distanceTo(centroid);
    if (distance < min_distance)
    {
      min_distance = distance;
      min_distance_id = i;
    }
  }
  PCL_INFO("Min distance: %2.3f.", min_distance);
  if (min_distance > merge_threshold_)
  {
    PCL_INFO("   >> new object <<\n");
    objects_.push_back(std::make_shared<Object>(cluster));
  }
  else
  {
    PCL_INFO("    merge with #%i\n", min_distance_id);
    objects_[min_distance_id]->mergeCluster(cluster);
  }
}

#endif
