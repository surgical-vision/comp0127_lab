#include "ros/ros.h"
#include "cw2q4/youbotKine.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"

#include "boost/foreach.hpp"

trajectory_msgs::JointTrajectoryPoint traj_pt;

//TODO: You can design your code to achieve the q6 task however you want as long as it stays in this file and it runs.

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_cw2");

    youbot_kinematic youbot;

    youbot.init();

    int checkpoint_data = atoi(argv[1]);
    int dt = 0.01; //Maybe change here.

    if (checkpoint_data == 1)
    {
        std::cout << "run q6a" << std::endl;
        //Load q6a data
    }
    else if (checkpoint_data == 2)
    {
        std::cout << "run q6b" << std::endl;
        //Load q6b data
    }

    while(ros::ok())
    {
        ros::spinOnce();
        usleep(10);
    }

    return 123;
}
