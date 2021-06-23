cmake_minimum_required(VERSION 2.8.3)
project(sol_lab2task3)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
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

add_executable(sol_lab2task3_pub src/publisher_node.cpp)
target_link_libraries(sol_lab2task3_pub ${catkin_LIBRARIES})

add_executable(sol_lab2task3_sub src/subscriber_node.cpp)
target_link_libraries(sol_lab2task3_sub ${catkin_LIBRARIES})
