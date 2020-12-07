#include <kdl_kine/kdl_kine_solver.h>
#include <string>

void robot_kinematic::init(std::string base_link, std::string ee_link)
{

    subscriber_joint_state = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &robot_kinematic::joint_state_callback, this);

    setup_kdl_chain();

    kdl_tree.getChain(base_link, ee_link, kine_chain);

    NrJoints = kine_chain.getNrOfJoints();

    KDL_joint = KDL::JntArray(NrJoints);

    current_joint_position.resize(NrJoints);
    current_joint_velocity.resize(NrJoints);
}

void robot_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    for (int i = 0; i < NrJoints; i++)
    {
        current_joint_position.data(i) = q->position.at(i);
        current_joint_velocity.data(i) = q->velocity.at(i);
    }
}


void robot_kinematic::setup_kdl_chain() {
    //Take physical parameters from the parameter "robot_description" as defined in the launch file. This will trace back to the urdf file.
    if (!kdl_parser::treeFromParam("robot_description", kdl_tree))
        ROS_ERROR("Failed to construct kdl tree");
}

KDL::Frame robot_kinematic::KDLfkine(KDL::JntArray current_joint_position)
{
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(this->kine_chain);
    KDL::Frame pose;
    fk_solver.JntToCart(current_joint_position, pose, NrJoints);
    return pose;
}

KDL::Jacobian robot_kinematic::KDLjacob(KDL::JntArray current_joint_position)
{
    KDL::ChainJntToJacSolver jac_solver = KDL::ChainJntToJacSolver(this->kine_chain);
    KDL::Jacobian jac = KDL::Jacobian(this->kine_chain.getNrOfJoints());
    jac_solver.JntToJac(current_joint_position, jac);
    return jac;
}

KDL::JntArray robot_kinematic::inverse_kinematics_closed(KDL::Frame desired_pose)
{

    KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(this->kine_chain);
    KDL::JntArray required_joint = KDL::JntArray(this->kine_chain.getNrOfJoints());
    ik_solver.CartToJnt(this->current_joint_position, desired_pose, required_joint);
    return required_joint;
}

MatrixXd robot_kinematic::getB(KDL::JntArray joint_val)
{
    KDL::JntSpaceInertiaMatrix KDL_B = KDL::JntSpaceInertiaMatrix(NrJoints);
    KDL::ChainDynParam dyn = KDL::ChainDynParam(kine_chain, KDL::Vector(0, 0, -9.8));

    dyn.JntToMass(joint_val, KDL_B);

    return KDL_B.data;
}


VectorXd robot_kinematic::getC(KDL::JntArray joint_val, KDL::JntArray joint_vel)
{


    KDL::JntArray KDL_C = KDL::JntArray(NrJoints);


    KDL::ChainDynParam dyn = KDL::ChainDynParam(kine_chain, KDL::Vector(0, 0, -9.8));


    dyn.JntToCoriolis(joint_val, joint_vel, KDL_C);

    return KDL_C.data;
}


VectorXd robot_kinematic::getG(KDL::JntArray joint_val)
{
    KDL::ChainDynParam dyn = KDL::ChainDynParam(kine_chain, KDL::Vector(0, 0, -9.8));

    KDL::JntArray gravity = KDL::JntArray(NrJoints);

    dyn.JntToGravity(joint_val, gravity);

    return gravity.data;
}
