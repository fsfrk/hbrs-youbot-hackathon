#include "object_tracker.h"
#include "occupancy_octree_object.h"
#include "bounding_box_object.h"

template class ObjectTracker<BoundingBoxObject>;
template class ObjectTracker<OccupancyOctreeObject>;
