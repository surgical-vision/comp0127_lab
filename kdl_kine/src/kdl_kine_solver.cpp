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
}

void robot_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    for (int i = 0; i < NrJoints; i++)
        this->current_joint_position.data(i) = q->position.at(i);
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

MatrixXd robot_kinematic::getB(VectorXd joint_val)
{
    KDL::JntArray q;
    KDL::JntSpaceInertiaMatrix KDL_B = KDL::JntSpaceInertiaMatrix(NrJoints);
    KDL::ChainDynParam dyn = KDL::ChainDynParam(kine_chain, KDL::Vector(0, 0, -9.8));

    q.resize(NrJoints);
    for (int i = 0; i < NrJoints; i++)
        q.data(i) = joint_val(i);

    dyn.JntToMass(q, KDL_B);

    return KDL_B.data;
}


VectorXd robot_kinematic::getC(VectorXd joint_val, VectorXd joint_vel)
{

    KDL::JntArray q, q_vel;
    KDL::JntArray KDL_C = KDL::JntArray(NrJoints);


    KDL::ChainDynParam dyn = KDL::ChainDynParam(kine_chain, KDL::Vector(0, 0, -9.8));

    q.resize(NrJoints);
    q_vel.resize(NrJoints);
    for (int i = 0; i < NrJoints; i++){
        q.data(i) = joint_val(i);
        q_vel.data(i) = joint_vel(i);
    }


    dyn.JntToCoriolis(q, q_vel, KDL_C);

    return KDL_C.data;
}


VectorXd robot_kinematic::getG(VectorXd joint_val)
{
    KDL::ChainDynParam dyn = KDL::ChainDynParam(kine_chain, KDL::Vector(0, 0, -9.8));

    KDL::JntArray q;
    KDL::JntArray gravity = KDL::JntArray(NrJoints);

    q.resize(NrJoints);
    for (int i = 0; i < NrJoints; i++)
        q.data(i) = joint_val(i);

    dyn.JntToGravity(q, gravity);

    return gravity.data;
}
