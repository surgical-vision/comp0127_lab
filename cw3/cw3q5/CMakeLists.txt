cmake_minimum_required(VERSION 2.8.3)
project(cw3q5)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cw3q2
  rosbag
  trajectory_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cw3q5a_node src/cw3q5a.cpp)
add_executable(cw3q5b_node src/cw3q5b.cpp)

add_dependencies(cw3q5a_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cw3q5b_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cw3q5a_node
  ${catkin_LIBRARIES}
)

target_link_libraries(cw3q5b_node
  ${catkin_LIBRARIES}
)

