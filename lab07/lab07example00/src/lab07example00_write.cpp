#include "ros/ros.h"
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "lab7example00_write");

    rosbag::Bag bag;
    bag.open(MY_BAG_PATH, rosbag::bagmode::Write);

    ros::NodeHandle nh;

    std_msgs::String str;
    str.data = std::string("foo");

    std_msgs::Int32 i;
    i.data = 42;

    bag.write("chatter", ros::Time::now(), str);
    bag.write("numbers", ros::Time::now(), i);


    ros::spinOnce();

    bag.close();
    return 5;
}