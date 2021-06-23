#include "ros/ros.h"
//TODO: Include the header file rotate_point service (look in the package example_msg_srv)

//TODO: Complete this function
bool rotate_point_3d();

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rotate_point_server");

    ros::NodeHandle nh;

    //TODO: Initialise the service 'rotate_point' and have it calls the function 'rotate_point_3d'
    ros::spin();

    return 1;
}
