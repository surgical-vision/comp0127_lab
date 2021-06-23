#include "ros/ros.h"
//TODO: include the library for a msg for an array integer
#include "std_msgs/Float64MultiArray.h"

//TODO: Complete the callback function. Your callback function should compute the sum (1 + 1/2 + 1/3 +... + 1/n) and print the output on the screen
void subscribeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    double sum = 0;

    for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	sum = sum + 1.0 / *it;

    printf("Current sum is: %.4f\n", sum);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    //TODO: Define your subscriber and link this with the callback function.
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/chatter", 1000, subscribeCallback);

    ros::spin();

    return 0;
}
