#ifndef DOMINANT_PLANE_EXTRACTOR_H_
#define DOMINANT_PLANE_EXTRACTOR_H_

#include "aliases.h"

/** Interface for the classes that analyze the scene and find the largest planar region. */
class DominantPlaneExtractor
{

public:

  typedef std::shared_ptr<DominantPlaneExtractor> Ptr;
  typedef std::unique_ptr<DominantPlaneExtractor> UPtr;

  DominantPlaneExtractor()
  : shrink_plane_polygon_by_(0.0)
  { }

  virtual void extract(PlanarPolygon& planar_polygon) = 0;

  virtual void setPlaneConstraints(const Eigen::Vector3f& normal, double angle_threshold) = 0;

  virtual void removePlaneConstraints() = 0;

  inline void setInputCloud(const PointCloud::ConstPtr &cloud)
  {
    input_ = cloud;
  }

  inline PointCloud::ConstPtr getInputCloud() const
  {
    return input_;
  }

  inline void setShrinkPlanePolygonBy(double meters)
  {
    shrink_plane_polygon_by_ = meters;
  }

  inline double getShrinkPlanePolygonBy() const
  {
    return shrink_plane_polygon_by_;
  }

  virtual ~DominantPlaneExtractor()
  {
    input_.reset();
  }

protected:

  static void shrinkPlanarPolygon(PlanarPolygon& planar_polygon, double shrink_by);

  PointCloud::ConstPtr input_;

  double shrink_plane_polygon_by_;

};

#endif
