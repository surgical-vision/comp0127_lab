#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_kdl/tf2_kdl.h>
#include <sensor_msgs/JointState.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>






double a[6] = {0.0, 0.265699, 0.03, 0.0, 0.0, 0.0};
double alpha[6] = {-M_PI_2, 0.0, -M_PI_2, -M_PI_2, -M_PI_2, 0.0};
double d[6] = {0.159, 0.0, 0.0, 0.258, 0.0, -0.123};
double theta[6] = {0.0, -M_PI_2 + atan(0.03/0.264), -atan(0.03/0.264), 0.0, 0.0, 0.0};






KDL::Chain make_chain(void){
    KDL::Chain c;
    for (int i=0;i<6;i++){
        c.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a[i], alpha[i], d[i], theta[i])));
    }
    return c;
}


void fkine(const sensor_msgs::JointState::ConstPtr& joint_msg, tf2_ros::TransformBroadcaster br, KDL::Chain chain) {

    Eigen::Matrix4d A;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
    KDL::ChainFkSolverPos_recursive FkSolver= KDL::ChainFkSolverPos_recursive(chain);

    geometry_msgs::TransformStamped transform[6];
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray JointPositions = KDL::JntArray(nj);


    transform[0].header.frame_id = transform[1].header.frame_id = transform[2].header.frame_id =
    transform[3].header.frame_id = transform[4].header.frame_id = transform[5].header.frame_id = "world";

    transform[0].header.stamp = transform[1].header.stamp = transform[2].header.stamp =
    transform[3].header.stamp = transform[4].header.stamp = transform[5].header.stamp = ros::Time::now();

    transform[0].child_frame_id = "kdl_link_1";
    transform[1].child_frame_id = "kdl_link_2";
    transform[2].child_frame_id = "kdl_link_3";
    transform[3].child_frame_id = "kdl_link_4";
    transform[4].child_frame_id = "kdl_link_5";
    transform[5].child_frame_id = "kdl_link_6";

    for (int i = 0; i < nj; i++)
    {
        
        if ((i == 5) || (i == 4)){
            JointPositions(i) = -joint_msg->position.at(i);
        }
        else{
            JointPositions(i) = joint_msg->position.at(i);
        }
    }
    KDL::Frame FrameT;
    for (int i = 0; i < nj; i++)
    {
        FkSolver.JntToCart(JointPositions,FrameT, i+1);

        Eigen::Affine3d T_affine;

        T_affine.matrix() = T;

        geometry_msgs::TransformStamped T_buffer = tf2::kdlToTransform(FrameT);

        transform[i].transform = T_buffer.transform;

        br.sendTransform(transform[i]);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotis_fkine_node");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster br;
    KDL::Chain chain = make_chain();


    ros::Subscriber joint_sub_standard = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, boost::bind(fkine, _1, br, chain));

    ros::spin();
}
