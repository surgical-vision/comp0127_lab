#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"

#include "kdl_kine/kdl_kine_solver.h"

#include "boost/foreach.hpp"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "lab7youbot_traj");

    rosbag::Bag mybag;

    trajectory_msgs::JointTrajectory my_traj;
    trajectory_msgs::JointTrajectoryPoint my_pt;


    //TODO: Initialise the bags and read the topic "my_transform" from the data
    //TODO: Do the inverse kinematic to those transform to get the corresponding joint.
    //TODO: Publish the JointTrajectory message to the corresponding topic to make the robot moves.

    //This sleep varies in different machines. This is to prevent the publisher destroying itself before even publishes one trajectory.
    sleep(5);

    mybag.close();
    return 5;
}
