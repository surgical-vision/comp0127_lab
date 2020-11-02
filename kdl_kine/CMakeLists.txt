cmake_minimum_required(VERSION 2.8.3)
project(kdl_kine)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        kdl_parser
        sensor_msgs
        )


find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
find_package(orocos_kdl REQUIRED)

set(HEADER_FILES
        include/kdl_kine/kdl_kine_solver.h)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES kdl_kine
        CATKIN_DEPENDS roscpp rospy sensor_msgs
        DEPENDS orocos_kdl EIGEN3 kdl_parser
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library

add_library(${PROJECT_NAME}
        src/kdl_kine_solver.cpp
        ${HEADER_FILES}
        )