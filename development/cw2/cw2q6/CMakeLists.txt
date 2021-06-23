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
  roscpp
  rospy
  sensor_msgs
  tf2_ros
  trajectory_msgs
)

add_definitions(-DMY_BAG_A="${PROJECT_SOURCE_DIR}/bags/data1.bag")
add_definitions(-DMY_BAG_B="${PROJECT_SOURCE_DIR}/bags/data2.bag")
##add_definitions(-DMY_BAG_C="${PROJECT_SOURCE_DIR}/bags/data3.bag")

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/cw2q6_node.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
