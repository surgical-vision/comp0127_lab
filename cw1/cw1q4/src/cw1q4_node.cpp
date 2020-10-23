#include "ros/ros.h"
//TODO: Include the header file for the three services
//TODO: Complete these three functions
bool convert_quat2zyx();
bool convert_quat2rodrigues();
bool convert_rotmat2quat();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_converter");
    ros::NodeHandle nh;

    //TODO: Define three services

    ros::spin();
    return 5;
}
