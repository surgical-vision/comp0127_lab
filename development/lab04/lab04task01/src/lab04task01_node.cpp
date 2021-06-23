#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

//TODO: Fill in the DH Parameters
double a[4] = {0.0, 0.0, 0.0, 0.0};
double alpha[4] = {0.0, 0.0, 0.0, 0.0};
double d[4] = {0.0, 0.0, 0.0, 0.0};
double theta[4] = {0.0, 0.0, 0.0, 0.0};

//TODO: Fill in this function
Eigen::Matrix4d fkine_standard(double a, double alpha, double d, double theta)
{
    Eigen::Matrix4d A;

    return A;
}

void fkine(const sensor_msgs::JointState::ConstPtr& joint_msg, tf2_ros::TransformBroadcaster br) {

    //TODO: Fill in the callback function
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "open_fkine_node");
    ros::NodeHandle nh;
    //TODO: Initialise a broadcaster and subscriber.

    ros::spin();
}