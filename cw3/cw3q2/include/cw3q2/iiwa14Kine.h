#ifndef CW3Q1_IIWA14KINE_H_H
#define CW3Q1_IIWA14KINE_H_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

using namespace Eigen;

class iiwa14_kinematic
{
protected:
    ros::NodeHandle nh;
    ros::Publisher traj_publisher;
    ros::Subscriber joint_state_sub;
    tf2_ros::TransformBroadcaster pose_br;
    double joint_limit_min[7];
    double joint_limit_max[7];
    Matrix4d current_pose;
    double current_joint_position[7];
    MatrixXd link_cm, translation_vec, Ixyz;
    VectorXd mass;
    double g;
    double X_alpha[7];
    double Y_alpha[7];
    double DH_params[7][4];


public:
    void init();
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);
    Matrix4d forward_kine(VectorXd joint_val, int frame);
    void broadcast_pose(Matrix4d pose);
    MatrixXd get_jacobian(VectorXd joint_val);
    MatrixXd inverse_kine_closed_form(Matrix4d pose);
    VectorXd inverse_kine_ite(Matrix4d pose, VectorXd joint_val);
    Matrix4d dh_matrix_standard(double a, double alpha, double d, double theta);
    MatrixXd getB(VectorXd joint_val);
    MatrixXd getC(VectorXd joint_val, VectorXd joint_vel);
    VectorXd getG(VectorXd joint_val);
    MatrixXd forward_kine_cm(VectorXd joint_val, int frame);
    MatrixXd get_jacobian_cm(VectorXd joint_val, int frame);

};

#endif //CW3Q1_IIWA14KINE_H_H
