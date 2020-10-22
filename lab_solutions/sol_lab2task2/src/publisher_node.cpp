#include "ros/ros.h"
//TODO: include the library for a msg for an array integer
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    //TODO: Define a nodehandle and a publisher
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/chatter", 1000);

    ros::Rate loop_rate(10);
    std_msgs::Float64MultiArray msg;

    int count = 1;

    while (ros::ok())
    {

        //TODO: For every loop, keep appending the array with the variable 'count' (If count = 4, an output array should be [1, 2, 3, 4])
        msg.data.push_back(count);
        //TODO: Publish the message.
        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }


    return 0;
}
