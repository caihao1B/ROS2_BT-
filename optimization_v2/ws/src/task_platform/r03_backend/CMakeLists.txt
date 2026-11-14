cmake_minimum_required(VERSION 3.16)
project(r03_path_backend)
set(CMAKE_CXX_STANDARD 17)
# The installed ROS OMPL config inherits an invalid vendor Eigen prefix.
# Resolve the installed headers/library directly without modifying system packages.
find_path(OMPL_INCLUDE ompl/base/spaces/ReedsSheppStateSpace.h PATHS /opt/ros/jazzy/include/ompl-1.7 REQUIRED)
find_library(OMPL_LIBRARY NAMES ompl PATHS /opt/ros/jazzy/lib/x86_64-linux-gnu REQUIRED)
find_path(EIGEN_INCLUDE Eigen/Core PATH_SUFFIXES eigen3 REQUIRED)
add_executable(r03_reeds_shepp reeds_shepp.cpp)
target_include_directories(r03_reeds_shepp PRIVATE ${OMPL_INCLUDE} ${EIGEN_INCLUDE})
target_link_libraries(r03_reeds_shepp ${OMPL_LIBRARY})
