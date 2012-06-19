#include <boost/format.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <dynamic_reconfigure/server.h>

#include "raw_perception_tests/TabletopClusterExtractorTestConfig.h"
#include "organized_dominant_plane_extractor.h"
#include "ransac_dominant_plane_extractor.h"
#include "tabletop_cluster_extractor.h"
#include "test_base.hpp"

#include <pcl/filters/passthrough.h>

class TabletopClusterExtractorTest : public TestBase
{

public:

  TabletopClusterExtractorTest(PointCloud::ConstPtr cloud, const char* method)
  : TestBase()
  , cloud_(cloud)
  {
    if (std::string("organized").compare(method) == 0)
      dpe_.reset(new OrganizedDominantPlaneExtractor);
    else if (std::string("ransac").compare(method) == 0)
      dpe_.reset(new RansacDominantPlaneExtractor);
    else
    {
      PCL_ERROR("Unsupported dominant plane extraction method: %s\n", method);
      throw std::runtime_error("Usupported method.");
    }
    extractPlanarPolygon();
    tce_.reset(new TabletopClusterExtractor());
    server_.setCallback(boost::bind(&TabletopClusterExtractorTest::reconfigure_callback, this, _1, _2));
  }

private:

  void extractPlanarPolygon()
  {
    planar_polygon_.reset(new PlanarPolygon);
    dpe_->setInputCloud(cloud_);
    dpe_->setShrinkPlanePolygonRatio(0.10);
    MEASURE_RUNTIME(dpe_->extract(*planar_polygon_), "Plane extraction");
    std::cout << "Number of points in plane contour: " << planar_polygon_->getContour().size() << std::endl;
    std::cout << "Plane coefficients:\n" << planar_polygon_->getCoefficients() << std::endl;
  }

  virtual void process()
  {
    std::vector<PointCloud::Ptr> clusters;
    tce_->setInputCloud(cloud_);
    tce_->setTablePolygon(planar_polygon_);
    MEASURE_RUNTIME(tce_->extract(clusters), "Cluster extraction");

    ROS_INFO("Number of clusters: %li.", clusters.size());

    viewer_.removeAllPointClouds(0);
    viewer_.removeAllShapes(0);

    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

    int i = 0;
    for (const PointCloud::Ptr& cluster : clusters)
    {
      ROS_INFO("Cluster %i: pts %li", i, cluster->points.size());
      std::string name = boost::str(boost::format("cluster_%02i") % i);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cluster, red[i % 6], grn[i % 6], blu[i % 6]);
      viewer_.addPointCloud<PointT>(cluster, single_color, name);
      viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
      i++;
    }

    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_, 0, 255, 0);
    viewer_.addPointCloud<PointT>(cloud_, single_color, "cloud");

    displayPlanarPolygon(*planar_polygon_);
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

  void reconfigure_callback(raw_perception_tests::TabletopClusterExtractorTestConfig &config, uint32_t level)
  {
    tce_.reset(new TabletopClusterExtractor(config.object_min_height,
                                            config.object_max_height,
                                            config.object_cluster_tolerance,
                                            config.object_cluster_min_size,
                                            config.object_cluster_max_size));
    process();
  }

  PointCloud::ConstPtr cloud_;
  PlanarPolygonPtr planar_polygon_;

  std::unique_ptr<DominantPlaneExtractor> dpe_;
  std::unique_ptr<TabletopClusterExtractor> tce_;

  dynamic_reconfigure::Server<raw_perception_tests::TabletopClusterExtractorTestConfig> server_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tabletop_cluster_extractor_test");

  if (argc != 3 && argc != 5)
  {
    ROS_ERROR("Usage: %s <filename.pcd> <ransac|organized> [min_z max_z]", argv[0]);
    return 1;
  }

  PointCloud::Ptr cloud(new PointCloud);
  pcl::io::loadPCDFile(argv[1], *cloud);

  if (argc == 5)
  {
    float min_z = boost::lexical_cast<float>(argv[3]);
    float max_z = boost::lexical_cast<float>(argv[4]);
    pcl::PassThrough<PointT> pass_through;
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(min_z, max_z);
    pass_through.setKeepOrganized(true);
    pass_through.setInputCloud(cloud);
    pass_through.filter(*cloud);
    ROS_INFO("Keep points in %.2f to %.2f range.", min_z, max_z);
  }

  TabletopClusterExtractorTest clustering(cloud, argv[2]);
  clustering.run();

  return 0;
}
