#include "ros/ros.h"
//TODO: Include the header file rotate_point service (look in the package example_msg_srv)

int main(int argc, char **argv) {

    ros::init(argc, argv, "rotate_point_client");

    ros::NodeHandle nh;

    //TODO: Initialise the client and makes sure that it corresponds to the correct service

    //TODO: Initialise the service message

    while (nh.ok())
    {

        //TODO: Generate a random point
        //TODO: Generate a random quaternion (make sure its norm is 1)

        //TODO: Send a request.
	//TODO: Show the computational time between requests (from sending a request and getting a response).
        //TODO: Change the output value
        ROS_INFO("The input point (x, y, z) = (%.2f, %.2f, %.2f)\n", 0.0, 0.0, 0.0);
        ROS_INFO("The output point (x, y, z) = (%.2f, %.2f, %.2f)\n", 0.0, 0.0, 0.0);

        usleep(10000);
    }

    return 1;
}
