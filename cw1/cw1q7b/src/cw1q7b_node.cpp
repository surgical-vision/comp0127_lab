#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_broadcaster.h"
#include "Eigen/Dense"

//TODO: Complete the forward kinematic routine using Eigen. The function should compute the transformation between each
//frame and use tf broadcaster to publish the transformation.
void forward_kinematic();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forward_kinematic_node");
    ros::NodeHandle nh;

    //TODO: Initialise a subscriber to the topic "/joint_states" and its callback function forward_kinematic
}
