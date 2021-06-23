#include <cw2q4/youbotKine.h>

void youbot_kinematic::init()
{
    DH_params[0][0] = -0.033;   DH_params[1][0] = 0.155;  DH_params[2][0] = 0.135; DH_params[3][0] = -0.002;  DH_params[4][0] = 0.0;
    DH_params[0][1] = M_PI_2;   DH_params[1][1] = 0.0;    DH_params[2][1] = 0.0;   DH_params[3][1] = M_PI_2;  DH_params[4][1] = M_PI;
    DH_params[0][2] = 0.145;    DH_params[1][2] = 0.0;    DH_params[2][2] = 0.0;   DH_params[3][2] = 0.0;     DH_params[4][2] = -0.185;
    DH_params[0][3] = M_PI;     DH_params[1][3] = M_PI_2; DH_params[2][3] = 0.0;   DH_params[3][3] = -M_PI_2; DH_params[4][3] = M_PI;

    joint_offset[0] = 170*M_PI/180;
    joint_offset[1] = 65*M_PI/180;
    joint_offset[2] = -146*M_PI/180;
    joint_offset[3] = 102.5*M_PI/180;
    joint_offset[4] = 167.5*M_PI/180;

    joint_limit_min[0] = -169*M_PI/180;
    joint_limit_min[1] = -65*M_PI/180;
    joint_limit_min[2] = -150*M_PI/180;
    joint_limit_min[3] = -102.5*M_PI/180;
    joint_limit_min[4] = -167.5*M_PI/180;

    joint_limit_max[0] = 169*M_PI/180;
    joint_limit_max[1] = 90*M_PI/180;
    joint_limit_max[2] = 146*M_PI/180;
    joint_limit_max[3] = 102.5*M_PI/180;
    joint_limit_max[4] = 167.5*M_PI/180;

    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &youbot_kinematic::joint_state_callback, this);
}


void youbot_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    for(int i = 0; i < 5; i++)
        current_joint_position[i] = q->position.at(i);

    current_pose = forward_kine(current_joint_position, 5);
    broadcast_pose(current_pose);
}

Matrix4d youbot_kinematic::dh_matrix_standard(double a, double alpha, double d, double theta)
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


Matrix4d youbot_kinematic::forward_kine(double joint_val[], int frame)
{

    Matrix4d A;
    Matrix4d T = Matrix4d::Identity(4, 4);

    for(int i = 0;i < frame; i++)
    {
        A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], joint_val[i] + DH_params[i][3]);

        T = T * A;
    }

    return T;
}

Matrix4d youbot_kinematic::forward_kine_offset(double joint_val[], int frame)
{

    Matrix4d A;
    Matrix4d T = Matrix4d::Identity(4, 4);

    for(int i = 0;i < frame; i++)
    {
        if (i == 0)
            A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], joint_offset[i] - (joint_val[i] + DH_params[i][3]));
        else
            A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], (joint_val[i] + DH_params[i][3]) - joint_offset[i]);

        T = T * A;
    }

    return T;
}

void youbot_kinematic::broadcast_pose(Matrix4d pose)
{

    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose;

    geometry_msgs::TransformStamped T = tf2::eigenToTransform(pose_affine);

    T.header.stamp = ros::Time::now();
    T.header.frame_id = "base_link";
    T.child_frame_id = "arm_end_effector";

    pose_br.sendTransform(T);
}

MatrixXd youbot_kinematic::get_jacobian(double joint_val[])
{
    //TODO: Fill in this function to complete the question 4a
}

MatrixXd youbot_kinematic::inverse_kine_closed_form(Matrix4d pose)
{
    //Ignore this.
}


double* youbot_kinematic::inverse_kine_ite(Matrix4d pose, double joint_val[])
{
    //TODO: Fill in this function to complete the question 4b
}


bool youbot_kinematic::check_singularity(double joint_val[])
{
    //TODO: Fill in this function to complete the question 4c
}
