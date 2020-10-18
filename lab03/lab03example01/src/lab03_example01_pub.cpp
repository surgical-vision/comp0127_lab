#include <ros/ros.h>
#include <example_msg_srv/test_msg.h>

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_publisher");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<example_msg_srv::test_msg>("/publish_rotation", 100);

    ros::Rate loop_rate(10);

    example_msg_srv::test_msg my_msg;

    while (nh.ok())
    {
        srand(time(0));
        my_msg.rotx.data = fRand(-2.0, 2.0);
        srand(time(0));
        my_msg.roty.data = fRand(-1.0, 1.0);
        srand(time(0));
        my_msg.rotz.data = fRand(-1.5, 1.5);

        pub.publish(my_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

}
