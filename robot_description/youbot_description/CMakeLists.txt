cmake_minimum_required(VERSION 2.8.3)
project(youbot_description)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY controller meshes robots urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

