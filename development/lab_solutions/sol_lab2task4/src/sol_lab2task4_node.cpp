#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <math.h>

//double joint1[10] = {1.7768, 0.2057, 2.8906, 3.3794, 2.9092, 2.3643, 1.2801, 2.2591, 1.7753, 0.9860};
//double joint2[10] = {1.7252, 0.0778, 0.6767, 0.1128, 0.2373, 2.0121, 1.6978, 0.7748, 2.3218, 0.0842};
//double joint3[10] = {-1.2207, -1.9312, -1.8746, -1.0249, -0.9458, -1.4789, -1.2553, -1.2713, -1.5904, -1.8198};
//double joint4[10] = {0.9635, 1.3726, 1.2867, 0.5676, 0.4154, 1.7396, 1.3501, 1.1882, 1.0430, 0.7813};

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_generator");

    ros::NodeHandle nh;
    ros::Publisher j1_pub = nh.advertise<std_msgs::Float64>(
            "/EffortJointInterface_J1_controller/command", 1);
    ros::Publisher j2_pub = nh.advertise<std_msgs::Float64>(
            "/EffortJointInterface_J2_controller/command", 1);
    ros::Publisher j3_pub = nh.advertise<std_msgs::Float64>(
            "/EffortJointInterface_J3_controller/command", 1);
    ros::Publisher j4_pub = nh.advertise<std_msgs::Float64>(
            "/EffortJointInterface_J4_controller/command", 1);

    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;
    std_msgs::Float64 msg3;
    std_msgs::Float64 msg4;

    while (nh.ok()) {
	double t = ros::Time::now().toSec();

        msg1.data = 200*M_PI/180*sin(2*M_PI*t/10);
        msg2.data = 50*M_PI/180*sin(2*M_PI*t/12);
        msg3.data = -80*M_PI/180*sin(2*M_PI*t/15);
        msg4.data = 60*M_PI/180*sin(2*M_PI*t/11);

        j1_pub.publish(msg1);
        j2_pub.publish(msg2);
        j3_pub.publish(msg3);
        j4_pub.publish(msg4);

        ros::spinOnce();
        //sleep(2);
    }

    return 0;
}
