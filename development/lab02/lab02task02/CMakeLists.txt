cmake_minimum_required(VERSION 2.8.3)
project(lab02task02)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        )

catkin_package()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(publisher_node_ex_02 src/publisher_node.cpp)
target_link_libraries(publisher_node_ex_02 ${catkin_LIBRARIES})

add_executable(subscriber_node_ex_02 src/subscriber_node.cpp)
target_link_libraries(subscriber_node_ex_02 ${catkin_LIBRARIES})
