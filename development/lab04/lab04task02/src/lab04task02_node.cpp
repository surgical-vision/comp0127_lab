#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_eigen/tf2_eigen.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "open_tf_listener_node");

    ros::NodeHandle nh;

    ros::Rate rate(10);

    //TODO: Initialise the tf listener
    //TODO: Initialise the tf broadcaster
    while (nh.ok())
    {
        //TODO: Create a routine to get the transformation from "world" to "link5"
        //TODO: Apply the transformation such that the new frame is located at the point 0.05m away from the end-effector (pointing-out direction)
        //and has the same orientation.
        //TODO: Broadcast the frame

        rate.sleep();
    }
    return 52;
}