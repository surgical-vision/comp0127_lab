cmake_minimum_required(VERSION 2.8.3)
project(lab03task01)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  example_msg_srv
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lab03_task01
#  CATKIN_DEPENDS example_msg_srv roscpp rospy
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_srv_node src/lab03_task01_srv_node.cpp)
add_executable(${PROJECT_NAME}_req_node src/lab03_task01_req_node.cpp)

add_dependencies(${PROJECT_NAME}_srv_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_req_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_srv_node
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_req_node
        ${catkin_LIBRARIES}
        )
