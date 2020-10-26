cmake_minimum_required(VERSION 2.8.3)
project(lab04example03)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf2_ros
  tf2_kdl
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs
  DEPENDS orocos_kdl EIGEN3
)


include_directories(
  ${catkin_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/lab04example03_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
