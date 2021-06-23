#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

void subscribeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) //Callback for the subscriber "sub"
{

    printf("Incoming array: ["); //Print the message

    for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
        printf("%.0f ", *it);

    printf("]\n");

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener"); //Initialise the node handle for the node "listener".

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/chatter", 1000, subscribeCallback); //Initialise the subscriber subscribing to the topic "/chatter".
    ros::spin();

    return 0;
}
