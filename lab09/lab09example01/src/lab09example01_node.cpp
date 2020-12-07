#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/JointState.h>
#include "kdl_kine/kdl_kine_solver.h"
#include <Eigen/Dense>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "kuka_dynamics_node");
    ros::NodeHandle nh;

    robot_kinematic iiwa_kine;
    ros::Rate loop_rate(10);

    //Initiate the base link (where the kinematic chain starts) and the end-effector link (where it ends). Please look at the
    // urdf file of the respective robot arm for the names.
    std::string base_link_name = "iiwa_link_0";
    std::string ee_link_name = "iiwa_link_ee";

    iiwa_kine.init(base_link_name, ee_link_name);

    KDL::JntArray q, qdot;
    q.resize(7);
    qdot.resize(7);


    while (ros::ok())
    {
       //Because joint_state_publisher does not publish velocity, we manually generate random joint positions and velocities as input to B, C, g matrices.
        MatrixXd q_and_qdot = MatrixXd::Random(7, 2);
        for (int i = 0; i < 7; i++)
        {
            q.data(i) = q_and_qdot(i, 0);
            qdot.data(i) = q_and_qdot(i, 1);
        }
        
        std::cout << "The current joint position: " << std::endl;
        std::cout << q.data << std::endl;

        std::cout << "The current joint velocity: " << std::endl;
        std::cout << qdot.data << std::endl;

        std::cout << "The current pose: " << std::endl;
        std::cout << iiwa_kine.KDLfkine(q) << std::endl;
        
        std::cout << "B(q): " << std::endl;
        std::cout << iiwa_kine.getB(q) << std::endl;
        std::cout << "C(q, qdot): " << std::endl;
        std::cout << iiwa_kine.getC(q, qdot) << std::endl;
        std::cout << "g(q): " << std::endl;
        std::cout << iiwa_kine.getG(q) << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}
