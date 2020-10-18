#include <ros/ros.h>
#include <Eigen/Dense>
#include <example_msg_srv/test_msg.h>

void subscribeCallback(const example_msg_srv::test_msg::ConstPtr& msg)
{
    Eigen::Matrix3d rot_mat;
    Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3, 3);
    double theta = sqrt(pow(msg->rotx.data, 2) + pow(msg->roty.data, 2) + pow(msg->rotz.data, 2));
    double omega_x = msg->rotx.data/theta;
    double omega_y = msg->roty.data/theta;
    double omega_z = msg->rotz.data/theta;

    Eigen::Matrix3d K;
    K(0, 0) = K(1, 1) = K(2, 2) = 0.0;
    K(0, 1) = -omega_z;
    K(1, 0) = omega_z;
    K(0, 2) = omega_y;
    K(2, 0) = -omega_y;
    K(1, 2) = -omega_x;
    K(2, 1) = omega_x;

    rot_mat = I + sin(theta)*K + (1 - cos(theta))*K*K;
    printf("The incoming matrix is: \n");
    std::cout << rot_mat << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_subscriber");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/publish_rotation", 100, subscribeCallback);

    ros::spin();

}
