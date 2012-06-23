#ifndef COLORED_OCCUPANCY_OCTREE_OBJECT_H_
#define COLORED_OCCUPANCY_OCTREE_OBJECT_H_

#include <string>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include "aliases.h"
#include "occupancy_octree_object.h"

class ColoredOccupancyOctreeObject : public OccupancyOctreeObject
{

public:

    explicit ColoredOccupancyOctreeObject(const PointCloud::ConstPtr& cluster, double resolution = 0.0025);

    void mergeCluster(const PointCloud::ConstPtr& cluster);

    double getMeanColor() const { return boost::accumulators::mean(color_); }

    double getMedianColor() const { return boost::accumulators::median(color_); }

private:

    typedef boost::accumulators::tag::mean mean;
    typedef boost::accumulators::tag::median median;
    boost::accumulators::accumulator_set<double, boost::accumulators::stats<mean, median>> color_;

};

#endif
