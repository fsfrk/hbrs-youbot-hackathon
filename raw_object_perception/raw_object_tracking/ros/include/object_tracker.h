#ifndef OBJECT_TRACKER_H_
#define OBJECT_TRACKER_H_

#include <vector>

#include "aliases.h"

template<typename ObjectT>
class ObjectTracker
{

public:

  typedef ObjectT Object;
  typedef typename std::shared_ptr<Object> ObjectPtr;
  typedef typename std::vector<ObjectPtr> ObjectPtrVector;

  ObjectTracker();

  virtual ~ObjectTracker();

  void addCluster(const PointCloud::ConstPtr& cluster);

  inline void setMergeThreshold(double threshold) { merge_threshold_ = threshold; }

  inline double getMergeThreshold() const { return merge_threshold_; }

  inline const ObjectPtrVector& getObjects() const { return objects_; }

  inline void reset() { objects_.clear(); }

private:

  ObjectPtrVector objects_;

  double merge_threshold_;

};

#include "impl/object_tracker.hpp"

#endif
