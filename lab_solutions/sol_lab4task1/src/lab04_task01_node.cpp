#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

//TODO: Fill in the DH Parameters
double a[4] = {0.0, 0.13, 0.124, 0.07};
double alpha[4] = {-M_PI_2, 0.0, 0.0, 0.0};
double d[4] = {0.075, 0.0, 0.0, 0.0};
double theta[4] = {0.0, -M_PI_2 + atan(0.024/0.128), atan(0.128/0.024), 0.0};

//TODO: Fill in this function
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

void fkine(const sensor_msgs::JointState::ConstPtr& joint_msg, tf2_ros::TransformBroadcaster br) {
    //TODO: Fill in the callback function
    Eigen::Matrix4d A;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);

    geometry_msgs::TransformStamped transform[4];

    transform[0].header.frame_id = transform[1].header.frame_id = transform[2].header.frame_id =
    transform[3].header.frame_id =  "world";

    transform[0].header.stamp = transform[1].header.stamp = transform[2].header.stamp =
    transform[3].header.stamp = ros::Time::now();

    transform[0].child_frame_id = "fkine_link_1";
    transform[1].child_frame_id = "fkine_link_2";
    transform[2].child_frame_id = "fkine_link_3";
    transform[3].child_frame_id = "fkine_link_4";

    for (int i = 0; i < 4; i++)
    {
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
    ros::init(argc, argv, "open_fkine_node");
    ros::NodeHandle nh;
    //TODO: Initialise a broadcaster and subscriber.
    tf2_ros::TransformBroadcaster br;

    ros::Subscriber joint_sub_standard = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, boost::bind(fkine, _1, br));

    ros::spin();
}
