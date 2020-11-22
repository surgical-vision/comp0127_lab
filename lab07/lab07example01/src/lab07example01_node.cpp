#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"

#include "boost/foreach.hpp"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "lab7youbot_traj");

    rosbag::Bag mybag;

    //Specify the mode (Read/Write)
    //Check the definition of "MY_BAG_PATH" in the CMakeLists.txt
    mybag.open(MY_BAG_PATH, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("my_joint_states"));

    trajectory_msgs::JointTrajectory my_traj;
    trajectory_msgs::JointTrajectoryPoint my_pt;

    ros::NodeHandle nh;

    //Initliaise the publisher
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 5);

    //Initialise all the necessary variables. It is important that the joint names match with the ones described in the urdf file of the robot.
    my_traj.header.stamp = ros::Time::now();
    my_traj.joint_names.push_back("arm_joint_1");
    my_traj.joint_names.push_back("arm_joint_2");
    my_traj.joint_names.push_back("arm_joint_3");
    my_traj.joint_names.push_back("arm_joint_4");
    my_traj.joint_names.push_back("arm_joint_5");

    my_pt.positions.resize(5);
    my_pt.velocities.resize(5);

    rosbag::View view(mybag, rosbag::TopicQuery(topics));

    int tfs = 4;

    BOOST_FOREACH(rosbag::MessageInstance const m, view) //Go through every message in the loaded bag.
                {
                    sensor_msgs::JointState::ConstPtr J = m.instantiate<sensor_msgs::JointState>();
                    if (J != NULL)
                    {
                        if (J->position.size() != 0)
                        {
                            my_pt.time_from_start.sec = tfs;
                            for (int i = 0; i < 5;i++)
                            {
                                my_pt.positions.at(i) = J->position.at(i);
                                my_pt.velocities.at(i) = J->velocity.at(i); //Try commenting-uncommenting these two lines to see the difference in the trajectory.
                                my_pt.accelerations.at(i) = 0; //Try commenting-uncommenting these two lines to see the difference in the trajectory.
                            }
                            my_traj.points.push_back(my_pt); //Add velocity and position into the trajectory message

                            tfs = tfs + 4;
                        }
                    }

                }

    //This sleep varies in different machines. This is to prevent the publisher destroying itself before even publishes one trajectory.
    sleep(5);

    traj_pub.publish(my_traj);
    ros::spinOnce();

    mybag.close();
    return 5;
}
