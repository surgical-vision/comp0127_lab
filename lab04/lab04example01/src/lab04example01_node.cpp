#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

//Initialise all DH parameters
double a[6] = {0.0, 0.265699, 0.03, 0.0, 0.0, 0.0};
double alpha[6] = {-M_PI_2, 0.0, -M_PI_2, -M_PI_2, -M_PI_2, 0.0};
double d[6] = {0.159, 0.0, 0.0, 0.258, 0.0, -0.123};
double theta[6] = {0.0, -M_PI_2 + atan(0.03/0.264), -atan(0.03/0.264), 0.0, 0.0, 0.0};

Eigen::Matrix4d fkine_standard(double a, double alpha, double d, double theta)
{//Define a transformation in standard DH.
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

void fkine(const sensor_msgs::JointState::ConstPtr& joint_msg, tf2_ros::TransformBroadcaster br) {
//Forward kinematic
    Eigen::Matrix4d A;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);

    geometry_msgs::TransformStamped transform[6];

    transform[0].header.frame_id = transform[1].header.frame_id = transform[2].header.frame_id =
    transform[3].header.frame_id = transform[4].header.frame_id = transform[5].header.frame_id = "world";

    transform[0].header.stamp = transform[1].header.stamp = transform[2].header.stamp =
    transform[3].header.stamp = transform[4].header.stamp = transform[5].header.stamp = ros::Time::now();

    transform[0].child_frame_id = "fkine_link_1";
    transform[1].child_frame_id = "fkine_link_2";
    transform[2].child_frame_id = "fkine_link_3";
    transform[3].child_frame_id = "fkine_link_4";
    transform[4].child_frame_id = "fkine_link_5";
    transform[5].child_frame_id = "fkine_link_6";

    for (int i = 0; i < 6; i++)
    {

        if ((i == 5) || (i == 4))
            A = fkine_standard(a[i], alpha[i], d[i], -joint_msg->position.at(i) + theta[i]);
        else
            A = fkine_standard(a[i], alpha[i], d[i], joint_msg->position.at(i) + theta[i]);

        T = T * A;

        Eigen::Affine3d T_affine;

        T_affine.matrix() = T;

        geometry_msgs::TransformStamped T_buffer = tf2::eigenToTransform(T_affine);

        transform[i].transform = T_buffer.transform;

        br.sendTransform(transform[i]);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotis_fkine_node");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster br;

    ros::Subscriber joint_sub_standard = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, boost::bind(fkine, _1, br));

    ros::spin();
}
