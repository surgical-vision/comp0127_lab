#include "ros/ros.h"
//TODO: include the library for a msg for an array integer

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    //TODO: Define a nodehandle and a publisher

    ros::Rate loop_rate(10);

    int count = 0;

    while (ros::ok())
    {

        //TODO: For every loop, keep appending the array with the variable 'count' (If count = 4, an output array should be [1, 2, 3, 4])
        //TODO: Publish the message.
        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }


    return 0;
}

