#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    ros::NodeHandle nh; //Initialise the node handle

    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("/publish_point", 1000); //Initialise publisher

    ros::Rate loop_rate(10);

    int radius = 5;
    int period = 5;
    geometry_msgs::PointStamped msg;

    double starting_time =ros::Time::now().toSec();

    while (ros::ok())
    {

        msg.header.stamp = ros::Time::now();

        //Publish the position of a point running on a 5 unit radius-circle
        msg.point.x = radius * cos(2 * M_PI * (msg.header.stamp.toSec() - starting_time)/period);
        msg.point.y = radius * sin(2 * M_PI * (msg.header.stamp.toSec() - starting_time)/period);
        msg.point.z = 0;

        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}

