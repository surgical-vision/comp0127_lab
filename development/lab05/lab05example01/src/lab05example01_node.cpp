#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/JointState.h>
#include "kdl_kine/kdl_kine_solver.h"
#include <Eigen/Dense>

KDL::JntArray joint_values;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "robotis_jacob_node");
    ros::NodeHandle nh;

    robot_kinematic h_kine;
    ros::Rate loop_rate(10);

    //Initiate the base link (where the kinematic chain starts) and the end-effector link (where it ends). Please look at the
    // urdf file of the respective robot arm for the names.
    std::string base_link_name = "link1";
    std::string ee_link_name = "end_link";

    h_kine.init(base_link_name, ee_link_name);

    while (ros::ok())
    {
        std::cout << "The current pose: " << std::endl;
        std::cout << h_kine.KDLfkine(h_kine.current_joint_position) << std::endl;
        std::cout << "The current Jacobian: " << std::endl;
        std::cout << h_kine.KDLjacob(h_kine.current_joint_position).data << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}
