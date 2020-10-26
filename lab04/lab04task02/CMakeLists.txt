cmake_minimum_required(VERSION 2.8.3)
project(lab04task02)

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
)

find_package(Eigen3 REQUIRED)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/lab04task02_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
