cmake_minimum_required(VERSION 2.8.3)
project(cw1q4_srv)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  geometry_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  quat2zyx.srv
  quat2rodrigues.srv
)

## Generate messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs
)
