#include "ros/ros.h"
//TODO: Include the header files for three services defined in lab03task02srv

//TODO: Complete these three functions
bool convert_quat2zyx();
bool convert_quat2angleaxis();
bool convert_rotmat2quat();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_converter");
    ros::NodeHandle nh;

    //TODO: Define three services

    ros::spin();
    return 5;
}
