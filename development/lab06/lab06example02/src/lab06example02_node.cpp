#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/JointState.h>
#include "kdl_kine/kdl_kine_solver.h"
#include "lab06example01/hArmKine.h"
#include <Eigen/Dense>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "robotis_inverse_kine_node");
    ros::NodeHandle nh;

    robot_kinematic h_kine;
    hArm_kinematic h_kine_lab;
    ros::Rate loop_rate(10);

    //Initiate the base link (where the kinematic chain starts) and the end-effector link (where it ends). Please look at the
    // urdf file of the respective robot arm for the names.
    std::string base_link_name = "link1";
    std::string ee_link_name = "end_link";

    h_kine.init(base_link_name, ee_link_name);
    h_kine_lab.init();

    KDL::Frame fkine_kdl;
    KDL::JntArray ikine_solution_kdl;
    Matrix4d fkine_lab;
    
    while (ros::ok())
    {

        fkine_kdl = h_kine.KDLfkine(h_kine.current_joint_position);
        ikine_solution_kdl = h_kine.inverse_kinematics_closed(fkine_kdl);
        
        fkine_lab = h_kine_lab.forward_kine(h_kine_lab.current_joint_position, 6);

        std::cout << "The current pose (kdl): " << std::endl;
        std::cout << fkine_kdl << std::endl;
        std::cout << "The current inverse kinematic solution (kdl): " << std::endl;
        std::cout << ikine_solution_kdl.data << std::endl;

        std::cout << "The current pose (customised): " << std::endl;
        std::cout << fkine_lab << std::endl;
        std::cout << "The current inverse kinematic solution (customised): " << std::endl;
        std::cout << h_kine_lab.inverse_kine_closed_form(fkine_lab) << std::endl;


        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}
