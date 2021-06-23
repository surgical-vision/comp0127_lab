#include "ros/ros.h"
//TODO: include the library for a msg typed VectorStamped

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    //TODO: Define a nodehandle and a publisher

    ros::Rate loop_rate(10);

    double radius = 1.0;
    double period = 5.0;

    //TODO: Define your message

    double starting_time =ros::Time::now().toSec();

    while (ros::ok())
    {

        //TODO: Input the data into your message. Your message should be a position of a unit circle on the XZ-plane.

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}

