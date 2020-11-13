cmake_minimum_required(VERSION 2.8.3)
project(cw2q4)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

set(HEADER_FILES include/cw2q4/youbotKine.h)

catkin_python_setup()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES cw2q4
        CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2_ros rospy
        DEPENDS EIGEN3
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/youbotKine.cpp
        ${HEADER_FILES}
)
