#include "ros/ros.h"
//TODO: include the library for a msg typed VectorStamped

double x_noisy_pose = 0;
double z_noisy_pose = 0;

//TODO: complete your callback function. Your callback function should add the noisy numbers to the incoming x and z positions.
void subscribeCallback();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh;

    //TODO: Create two random numbers between 0 and 1.

    //TODO: Create a subscriber and links to the callback function.
    while (nh.ok())
    {
        printf("The current point is (%.3f, %.3f)\n", x_noisy_pose, z_noisy_pose);
        ros::spinOnce();
        //Sleep for 20usec to prevent flooding
        usleep(20);
    }

    return 0;
}