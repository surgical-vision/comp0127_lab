#include <cw3q2/iiwa14Kine.h>

void iiwa14_kinematic::init()
{
    //Delete this and fill DH parameters based on the xacro file (cw3/iiwa_description/urdf/iiwa14.xacro).
    for (int i = 0; i < 7;i++)
        for (int j = 0; j < 4;j++)
            DH_params[i][j] = 0.0;

    joint_limit_min[0] = -170*M_PI/180;
    joint_limit_min[1] = -120*M_PI/180;
    joint_limit_min[2] = -170*M_PI/180;
    joint_limit_min[3] = -120*M_PI/180;
    joint_limit_min[4] = -170*M_PI/180;
    joint_limit_min[5] = -120*M_PI/180;
    joint_limit_min[6] = -175*M_PI/180;

    joint_limit_max[0] = 170*M_PI/180;
    joint_limit_max[1] = 120*M_PI/180;
    joint_limit_max[2] = 170*M_PI/180;
    joint_limit_max[3] = 120*M_PI/180;
    joint_limit_max[4] = 170*M_PI/180;
    joint_limit_max[5] = 120*M_PI/180;
    joint_limit_max[6] = 175*M_PI/180;

    //The mass of each link.
    mass.resize(7);
    mass << 4, 4, 3, 2.7, 1.7, 1.8, 0.3;

    //Moment on inertia of each link.
    //Each row is (Ixx, Iyy, Izz) and Ixy = Ixz = Iyz = 0.
    Ixyz.resize(7, 3);
    Ixyz << 0.1, 0.09, 0.02,
            0.05, 0.018, 0.044,
            0.08, 0.075, 0.01,
            0.03, 0.01, 0.029,
            0.02, 0.018, 0.005,
            0.005, 0.0036, 0.0047,
            0.001, 0.001, 0.001;

    //gravity
    g = 9.8;

    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &iiwa14_kinematic::joint_state_callback, this);
}


void iiwa14_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    VectorXd J(7);

    for(int i = 0; i < 7; i++)
        J(i) = q->position.at(i);

    current_pose = forward_kine(J, 7);
    broadcast_pose(current_pose);
}

Matrix4d iiwa14_kinematic::dh_matrix_standard(double a, double alpha, double d, double theta)
{
    Matrix4d A;
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

void iiwa14_kinematic::broadcast_pose(Matrix4d pose)
{

    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose;

    geometry_msgs::TransformStamped T = tf2::eigenToTransform(pose_affine);

    T.header.stamp = ros::Time::now();
    T.header.frame_id = "iiwa_link_0";
    T.child_frame_id = "iiwa_ee";

    pose_br.sendTransform(T);
}

//Useful Transformation function.
Matrix4d T_translation(Vector3d t)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    for (int i = 0; i < 3; i++)
        T(i, 3) = t(i);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationZ(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta);
    T(1, 0) = sin(theta);
    T(1, 1) = cos(theta);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationY(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(0, 0) = cos(theta);
    T(0, 2) = sin(theta);
    T(2, 0) = -sin(theta);
    T(2, 2) = cos(theta);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationX(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(1, 1) = cos(theta);
    T(1, 2) = -sin(theta);
    T(2, 1) = sin(theta);
    T(2, 2) = cos(theta);
    return T;
}

Matrix4d iiwa14_kinematic::forward_kine(VectorXd joint_val, int frame)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    //Add offset from the iiwa platform.
    T(2, 3) = 0.1575;
    //TODO: Fill in this function to complete Q2.
    return T;
}

MatrixXd iiwa14_kinematic::forward_kine_cm(VectorXd joint_val, int frame)
{
    //TODO: Fill in this function to complete Q2.
}

MatrixXd iiwa14_kinematic::get_jacobian(VectorXd joint_val)
{
    //TODO: Fill in this function to complete Q2.
}

MatrixXd iiwa14_kinematic::get_jacobian_cm(VectorXd joint_val, int frame)
{
    //TODO: Fill in this function to complete Q2.
}

VectorXd iiwa14_kinematic::inverse_kine_ite(Matrix4d pose, VectorXd joint_val)
{
    //TODO: Fill in this function to complete Q2.
}

MatrixXd iiwa14_kinematic::inverse_kine_closed_form(Matrix4d pose)
{
    //TODO: Fill in this function to complete Q2.
}

MatrixXd iiwa14_kinematic::getB(VectorXd joint_val)
{
    //TODO: Fill in this function to complete Q2.
}

MatrixXd iiwa14_kinematic::getC(VectorXd joint_val, VectorXd joint_vel)
{
    //TODO: Fill in this function to complete Q2.
}

VectorXd iiwa14_kinematic::getG(VectorXd joint_val)
{
    //TODO: Fill in this function to complete Q2.
}
