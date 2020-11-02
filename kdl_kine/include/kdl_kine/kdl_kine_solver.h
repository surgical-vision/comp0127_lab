#ifndef KDL_KINE_KDL_KINE_SOLVER_H_H
#define KDL_KINE_KDL_KINE_SOLVER_H_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <sensor_msgs/JointState.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

using namespace Eigen;

class robot_kinematic
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber subscriber_joint_state;
    int NrJoints;


public:
    KDL::Frame KDL_pose;
    KDL::Chain kine_chain;
    KDL::JntArray KDL_joint;
    KDL::Tree kdl_tree;
    KDL::JntArray current_joint_position;


    void init(std::string base_link, std::string ee_link);
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);
    MatrixXd getB(VectorXd joint_val);
    VectorXd getC(VectorXd joint_val, VectorXd joint_vel);
    VectorXd getG(VectorXd joint_val);
    void setup_kdl_chain();
    KDL::Jacobian KDLjacob(KDL::JntArray current_joint_position);
    KDL::Frame KDLfkine(KDL::JntArray current_joint_position);
    KDL::JntArray inverse_kinematics_closed(KDL::Frame desired_pose);
};

#endif //KDL_KINE_KDL_KINE_SOLVER_H_H
