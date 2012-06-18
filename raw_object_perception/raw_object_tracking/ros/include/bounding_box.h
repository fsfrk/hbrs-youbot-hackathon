#ifndef BOUNDING_BOX_H_
#define BOUNDING_BOX_H_

#include <pcl/point_cloud.h>

namespace raw
{

class BoundingBox
{

  public:

    typedef Eigen::Vector3f Point;
    typedef std::vector<Point, Eigen::aligned_allocator<Point>> Points;

    inline const Point& getCenter() const { return center_; }

    inline const Points& getVertices() const { return vertices_; }

    inline float getLength() const { return length_; }

    inline float getWidth() const { return width_; }

    inline float getHeight() const { return height_; }

    inline float getVolume() const { return length_ * width_ * height_; }

    /** Create a bounding box around the cloud, restricting it to be parallel to the plane defined by the normal. */
    template<typename PointT>
    static BoundingBox create(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const Eigen::Vector3f& normal);

    /** Create a bounding box around the point vector, restricting it to be parallel to the plane defined by the normal. */
    template<typename PointT>
    static BoundingBox create(const typename pcl::PointCloud<PointT>::VectorType& points, const Eigen::Vector3f& normal);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    Point center_;
    Points vertices_;

    float length_;
    float width_;
    float height_;

};

}

#include "impl/bounding_box.hpp"

#endif
