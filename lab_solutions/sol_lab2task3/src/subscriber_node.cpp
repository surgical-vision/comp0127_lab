#include "ros/ros.h"
//TODO: include the library for a msg typed VectorStamped
#include "geometry_msgs/Vector3Stamped.h"

double x_noisy_pose = 0;
double z_noisy_pose = 0;

//TODO: complete your callback function. Your callback function should add the noisy numbers to the incoming x and z positions.
void subscribeCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg, int input1, int input2){
    //Add some offset
    x_noisy_pose = msg->vector.x + input1;
    z_noisy_pose = msg->vector.z - input2;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle nh;

    //TODO: Create two random numbers between 0 and 1.
    int test_input1 = rand() % 1;
    int test_input2 = rand() % 1;

    //TODO: Create a subscriber and links to the callback function.
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/publish_point", 1000,
                                                                    boost::bind(subscribeCallback, _1, test_input1, test_input2));

    while (nh.ok())
    {
        printf("The current point is (%.3f, %.3f)\n", x_noisy_pose, z_noisy_pose);
        ros::spinOnce();
        //Sleep for 20usec to prevent flooding
        usleep(20);
    }

    return 0;
}
