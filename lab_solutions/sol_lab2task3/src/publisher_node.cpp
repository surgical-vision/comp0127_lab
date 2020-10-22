#include "ros/ros.h"
//TODO: include the library for a msg typed VectorStamped
#include "geometry_msgs/Vector3Stamped.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    //TODO: Define a nodehandle and a publisher
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3Stamped>("/publish_point", 1000);

    ros::Rate loop_rate(10);

    double radius = 1.0;
    double period = 5.0;

    //TODO: Define your message
    geometry_msgs::Vector3Stamped msg;

    double starting_time =ros::Time::now().toSec();

    while (ros::ok())
    {

        //TODO: Input the data into your message. Your message should be a position of a unit circle on the XZ-plane.
        msg.header.stamp = ros::Time::now();

        msg.vector.x = radius * cos(2 * M_PI * (msg.header.stamp.toSec() - starting_time)/period);
        msg.vector.y = 0;
        msg.vector.z = radius * sin(2 * M_PI * (msg.header.stamp.toSec() - starting_time)/period);

        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}

