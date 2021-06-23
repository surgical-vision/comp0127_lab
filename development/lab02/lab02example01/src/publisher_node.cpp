#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker"); //Initialise the node handle for the node "talker".

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/chatter", 1000); //Initialise a publisher to the topic "/chatter" with a queue size of 1000

    ros::Rate loop_rate(10);
    std_msgs::Float64MultiArray msg; //Initialise the message (type = Float64MultiArray)

    int count = 0;

    while (ros::ok())
    {


        msg.data.push_back(count); //Add the variable "count" to the list

        pub.publish(msg); //Publish the data

        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }


    return 0;
}

