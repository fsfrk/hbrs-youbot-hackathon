#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <dynamic_reconfigure/server.h>

#include "raw_perception_tests/OrganizedDominantPlaneExtractorTestConfig.h"
#include "raw_perception_tests/RansacDominantPlaneExtractorTestConfig.h"
#include "organized_dominant_plane_extractor.h"
#include "ransac_dominant_plane_extractor.h"
#include "test_base.hpp"

class DominantPlaneExtractorTest : public TestBase
{

public:

    DominantPlaneExtractorTest(PointCloud::ConstPtr cloud)
    : TestBase()
    , cloud_(cloud)
    { }

protected:

    virtual void process()
    {
      PlanarPolygon planar_polygon;

      dpe_->setInputCloud(cloud_);
      MEASURE_RUNTIME(dpe_->extract(planar_polygon), "Plane extraction");

      std::cout << "Number of points in plane contour: " << planar_polygon.getContour().size() << std::endl;
      std::cout << "Plane coefficients:\n" << planar_polygon.getCoefficients() << std::endl;

      viewer_.removeAllPointClouds(0);
      viewer_.removeAllShapes(0);

      pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_, 0, 255, 0);
      viewer_.addPointCloud<PointT>(cloud_, single_color, "cloud");

      if (plane_normal_)
        displayNormal(*plane_normal_);

      displayPlanarPolygon(planar_polygon);
    }

    /** For the given PlanarPolygon draw a polyline through it points and also display the points themselves. */
    void displayPlanarPolygon(const PlanarPolygon& polygon)
    {
      PointCloud::Ptr contour(new PointCloud);
      contour->points = polygon.getContour();
      if (!contour->points.size())
        return;
      viewer_.addPolygon<PointT>(contour, 255, 0, 0, "polygon");
      pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(contour, 255, 0, 0);
      viewer_.addPointCloud(contour, red_color, "polygon_points");
      viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "polygon_points");
    }

    std::unique_ptr<DominantPlaneExtractor> dpe_;
    PointCloud::ConstPtr cloud_;
    std::unique_ptr<Eigen::Vector3f> plane_normal_;

};

class OrganizedDominantPlaneExtractorTest : public DominantPlaneExtractorTest
{

public:

  OrganizedDominantPlaneExtractorTest(PointCloud::ConstPtr cloud)
  : DominantPlaneExtractorTest(cloud)
  {
    server_.setCallback(boost::bind(&OrganizedDominantPlaneExtractorTest::reconfigure_callback, this, _1, _2));
    dpe_.reset(new OrganizedDominantPlaneExtractor);
  };

private:

  void reconfigure_callback(raw_perception_tests::OrganizedDominantPlaneExtractorTestConfig &config, uint32_t level)
  {
    dpe_.reset(new OrganizedDominantPlaneExtractor(config.normal_max_depth_change_factor,
                                                   config.normal_smoothing_size,
                                                   config.min_inliers,
                                                   pcl::deg2rad(config.angular_threshold),
                                                   config.distance_threshold,
                                                   config.maximum_curvature,
                                                   config.refinement_threshold,
                                                   config.refinement_depth_independent));
    if (config.apply_plane_constraints)
    {
      plane_normal_.reset(new Eigen::Vector3f(config.plane_normal_x, config.plane_normal_y, config.plane_normal_z));
      plane_normal_->normalize();
      double threshold = pcl::deg2rad(config.plane_normal_angular_threshold);
      dpe_->setPlaneConstraints(*plane_normal_, threshold);
    }
    else
    {
      plane_normal_.reset();
    }
    process();
  }

  dynamic_reconfigure::Server<raw_perception_tests::OrganizedDominantPlaneExtractorTestConfig> server_;

};

class RansacDominantPlaneExtractorTest : public DominantPlaneExtractorTest
{

public:

  RansacDominantPlaneExtractorTest(PointCloud::ConstPtr cloud)
  : DominantPlaneExtractorTest(cloud)
  {
    server_.setCallback(boost::bind(&RansacDominantPlaneExtractorTest::reconfigure_callback, this, _1, _2));
    dpe_.reset(new RansacDominantPlaneExtractor);
  };

private:

  void reconfigure_callback(raw_perception_tests::RansacDominantPlaneExtractorTestConfig &config, uint32_t level)
  {
    dpe_.reset(new RansacDominantPlaneExtractor(config.downsampling_leaf_size,
                                                config.normal_estimation_neighbors,
                                                config.sac_distance_threshold,
                                                config.sac_max_iterations,
                                                config.sac_normal_distance_weight,
                                                config.sac_optimize_coefficiens,
                                                config.sac_probability));
    if (config.apply_plane_constraints)
    {
      plane_normal_.reset(new Eigen::Vector3f(config.plane_normal_x, config.plane_normal_y, config.plane_normal_z));
      plane_normal_->normalize();
      double threshold = pcl::deg2rad(config.plane_normal_angular_threshold);
      dpe_->setPlaneConstraints(*plane_normal_, threshold);
    }
    else
    {
      plane_normal_.reset();
    }
    process();
  }

  dynamic_reconfigure::Server<raw_perception_tests::RansacDominantPlaneExtractorTestConfig> server_;

};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "dominant_plane_extractor_test");

  if (argc != 3)
  {
    PCL_ERROR("Usage: %s <filename.pcd> <ransac|organized>\n", argv[0]);
    return 1;
  }

  PointCloud::Ptr cloud(new PointCloud);
  pcl::io::loadPCDFile(argv[1], *cloud);

  std::unique_ptr<DominantPlaneExtractorTest> dpet;

  if (std::string("organized").compare(argv[2]) == 0)
    dpet.reset(new OrganizedDominantPlaneExtractorTest(cloud));
  else if (std::string("ransac").compare(argv[2]) == 0)
    dpet.reset(new RansacDominantPlaneExtractorTest(cloud));
  else
    ROS_ERROR("Unsupported dominant plane extraction method: %s\n", argv[2]);

  if (dpet)
    dpet->run();
  else
    return 1;

  return 0;
}
