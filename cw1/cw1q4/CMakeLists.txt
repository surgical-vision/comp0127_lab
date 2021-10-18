cmake_minimum_required(VERSION 2.8.3)
project(cw1q4)

find_package(catkin REQUIRED COMPONENTS
  cw1q4_srv
  rospy
)

catkin_package()

catkin_python_setup()
