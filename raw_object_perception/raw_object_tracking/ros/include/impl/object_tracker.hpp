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
  PCL_INFO("Add new cluster with %li points.\n", cluster->points.size());
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
  if (min_distance > merge_threshold_)
  {
    PCL_INFO(" [  new  ] could not merge with existing objects (min distance: %.3f)\n", min_distance);
    objects_.push_back(std::make_shared<Object>(cluster));
  }
  else
  {
    PCL_INFO(" [ merge ] with object #%i (distance: %.3f)\n", min_distance_id, min_distance);
    objects_[min_distance_id]->mergeCluster(cluster);
  }
  PCL_INFO("Total number of objects: %i.\n", objects_.size());
}

#endif
