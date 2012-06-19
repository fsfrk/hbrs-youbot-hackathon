#ifndef TEST_BASE_HPP_
#define TEST_BASE_HPP_

#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>

#define MEASURE_RUNTIME(function, description) \
        { \
          double ts1 = pcl::getTime(); \
          (function); \
          double ts2 = pcl::getTime(); \
          std::cout << (description) << " took " << ts2 - ts1 << " seconds." << std::endl; \
        }

/** Base class for perception test programs.
  * Provides PCLVisualizer initialization, infinite superloop (@ref run()), and several helper functions to display
  * different objects. */
class TestBase
{

public:

  TestBase()
  : viewer_("Viewer")
  {
    viewer_.setBackgroundColor(0, 0, 0);
    viewer_.addCoordinateSystem(1.0);
    viewer_.initCameraParameters();
  }

  virtual ~TestBase() { }

  void run()
  {
    process();
    while (!viewer_.wasStopped() && !ros::isShuttingDown())
    {
      viewer_.spinOnce(100);
      ros::spinOnce();
    }
  }

protected:

  virtual void process() = 0;

  void displayNormal(Eigen::Vector3f normal, Eigen::Vector3f start = Eigen::Vector3f::Zero())
  {
    pcl::PointXYZ pt1 = pcl::PointXYZ(start[0], start[1], start[2]);
    pcl::PointXYZ pt2 = pcl::PointXYZ(start[0] + (0.5 * normal[0]),
                                      start[1] + (0.5 * normal[1]),
                                      start[2] + (0.5 * normal[2]));
    viewer_.addArrow(pt2, pt1, 1.0, 0, 0, false, "normal");
  }

  pcl::visualization::PCLVisualizer viewer_;

};

#endif
