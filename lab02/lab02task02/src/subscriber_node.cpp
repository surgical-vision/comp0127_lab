#include "ros/ros.h"
//TODO: include the library for a msg for an array integer

//TODO: Complete the callback function. Your callback function should compute the sum (1 + 1/2 + 1/3 +... + 1/n) and print the output on the screen
void subscribeCallback();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    //TODO: Define your subscriber and link this with the callback function.

    ros::spin();

    return 0;
}