cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#...: compiler options :........................................................

#.... gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
        message(SEND_ERROR "You need GCC version 4.4 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
endif(CMAKE_COMPILER_IS_GNUCXX)

#.... treat warnings as errors
add_definitions(-Werror)
add_definitions(-Wno-error=unused-variable)
add_definitions(-Wno-error=unknown-pragmas)
add_definitions(-Wno-deprecated)

#...: default paths  :..........................................................

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/ros/include)

#...: vtk libraries for pcl visualization  :....................................

set(VTK_LIBS libvtkCommon.so libvtkFiltering.so libvtkRendering.so)

#...: target executables :......................................................

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#...: dominant_plane_extraction_test
rosbuild_add_executable(dominant_plane_extractor_test src/dominant_plane_extractor_test.cpp
)
target_link_libraries(dominant_plane_extractor_test ${VTK_LIBS}
                                                    scene_segmentation
)

#...: online_dominant_plane_extraction_test
rosbuild_add_executable(online_dominant_plane_extractor_test src/online_dominant_plane_extractor_test.cpp
)
target_link_libraries(online_dominant_plane_extractor_test scene_segmentation
)

#...: online_cluster_finder_test
rosbuild_add_executable(online_cluster_finder_test src/online_cluster_finder_test.cpp
)

#...: tabletop_cluster_extractor_test
rosbuild_add_executable(tabletop_cluster_extractor_test src/tabletop_cluster_extractor_test.cpp
)
target_link_libraries(tabletop_cluster_extractor_test ${VTK_LIBS}
                                                      scene_segmentation
)

#...: bounding_box_test
rosbuild_add_executable(bounding_box_test src/bounding_box_test.cpp
)
target_link_libraries(bounding_box_test ${VTK_LIBS}
)
