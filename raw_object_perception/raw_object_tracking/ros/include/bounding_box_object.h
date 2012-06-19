#ifndef BOUNDING_BOX_OBJECT_H_
#define BOUNDING_BOX_OBJECT_H_

#include "aliases.h"

class BoundingBoxObject
{

  public:

    explicit BoundingBoxObject(const PointCloud::ConstPtr& cluster);

    void mergeCluster(const PointCloud::ConstPtr& cluster);

    double distanceTo(const Eigen::Vector4f& point) const;

    inline const Eigen::Vector4f& getMinPoint() const { return min_point_; }

    inline const Eigen::Vector4f& getMaxPoint() const { return max_point_; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    Eigen::Vector4f min_point_;
    Eigen::Vector4f max_point_;
    Eigen::Vector4f box_center_;

};

#endif
