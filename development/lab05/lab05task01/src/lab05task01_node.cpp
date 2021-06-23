#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>

double a[6] = {0.0, 0.265699, 0.03, 0.0, 0.0, 0.0};
double alpha[6] = {-M_PI_2, 0.0, -M_PI_2, -M_PI_2, -M_PI_2, 0.0};
double d[6] = {0.159, 0.0, 0.0, 0.258, 0.0, -0.123};
double theta[6] = {0.0, -M_PI_2 + atan(0.03/0.264), -atan(0.03/0.264), 0.0, 0.0, 0.0};

Eigen::Matrix4d fkine_standard(double a, double alpha, double d, double theta)
{
    Eigen::Matrix4d A;
    A(3, 3) = 1.0;
    A(3, 2) = 0.0;
    A(3, 1) = 0.0;
    A(3, 0) = 0.0;

    A(0, 0) = cos(theta);
    A(0, 1) = -sin(theta)*cos(alpha);
    A(0, 2) = sin(theta)*sin(alpha);
    A(0, 3) = a * cos(theta);

    A(1, 0) = sin(theta);
    A(1, 1) = cos(theta)*cos(alpha);
    A(1, 2) = -cos(theta)*sin(alpha);
    A(1, 3) = a * sin(theta);

    A(2, 0) = 0.0;
    A(2, 1) = sin(alpha);
    A(2, 2) = cos(alpha);
    A(2, 3) = d;

    return A;
}

void compute_jacobian(const sensor_msgs::JointState::ConstPtr& joint_msg) {

    Eigen::Matrix4d A;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);

    //TODO: fill in the forward kinematic routine
    //TODO: Compute the Jacobian and print the matrix
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotis_jacob_node");
    ros::NodeHandle nh;

    ros::Subscriber joint_sub_standard = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, compute_jacobian);

    ros::spin();
}
