cmake_minimum_required(VERSION 2.8.3)
project(cw2q6)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cw2q4
  geometry_msgs
  rosbag
  rospy
  sensor_msgs
  tf2_ros
  trajectory_msgs
)

catkin_package()
catkin_python_setup()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
