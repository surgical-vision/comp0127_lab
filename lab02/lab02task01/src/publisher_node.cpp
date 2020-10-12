#include "ros/ros.h"
//TODO: include the library for a msg for an integer

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    //TODO: Define a nodehandle and a publisher.

    ros::Rate loop_rate(10);

    int count = 0;

    while (ros::ok())
    {

        //TODO: Create a message and publish it. Your message should be a random number between 0 and the variable 'count'

        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }


    return 0;
}

