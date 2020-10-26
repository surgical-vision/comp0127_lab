#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotis_tf_listener_node");

    ros::NodeHandle nh;

    ros::Rate rate(10);

    tf2_ros::Buffer tfBuffer; //Initialise tf 
    tf2_ros::TransformListener tfListener(tfBuffer); //Subscriber for tf topic

    while (nh.ok())
    {
        geometry_msgs::TransformStamped T;

        try
        {
            T = tfBuffer.lookupTransform("world", "link6", ros::Time(0)); //Look for a transformation that links the frame "world" and "link6" together
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        std::cout << T << std::endl;

        rate.sleep();
    }
    return 52;
}
