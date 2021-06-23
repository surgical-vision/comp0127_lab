#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

double x = 0;
double y = 0;

void subscribeCallback(const geometry_msgs::PointStamped::ConstPtr& msg, int input1, int input2) //Subscriber callback with additional arguments
{

    //Add some offset
    x = msg->point.x + input1;
    y = msg->point.y - input2;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh;

    int test_input1 = 10;
    int test_input2 = -10;

    //The template for the definition of the subscriber should specify the type of the topic you are subscribing and
    //should remain like this
    //boost::bind(subscribeFunction, _1, arg1, arg2, arg3, ..., argN)
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>("/publish_point", 1000,
                                                                    boost::bind(subscribeCallback, _1, test_input1, test_input2));
    while (nh.ok())
    {
        printf("The current point is (%.3f, %.3f)\n", x, y);
        ros::spinOnce();
        //Sleep for 20usec to prevent flooding
        usleep(20);
    }

    return 0;
}
