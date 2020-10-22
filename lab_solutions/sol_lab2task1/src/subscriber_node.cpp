#include "ros/ros.h"
//TODO: include the library for a msg for an integer
#include "std_msgs/Float64.h"

//TODO: complete the callback function. Your callback function should print out the incoming message.
void subscribeCallback(const std_msgs::Float64 msg)
{

    printf("Incoming number between 0 and the var count: %.0f", msg.data);

    printf("\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    //TODO: Define a nodehandle and a subscriber.
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/chatter", 1000, subscribeCallback);

    ros::spin();

    return 0;
}
