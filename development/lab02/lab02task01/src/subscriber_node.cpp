#include "ros/ros.h"
//TODO: include the library for a msg for an integer

//TODO: complete the callback function. Your callback function should print out the incoming message.
void subscribeCallback();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    //TODO: Define a nodehandle and a subscriber.
    ros::spin();

    return 0;
}