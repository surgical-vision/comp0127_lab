#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>

visualization_msgs::Marker robot_trail;
geometry_msgs::Point point_buffer;

void update_line(const gazebo_msgs::LinkStates::ConstPtr &pos)
{
    int L = pos->pose.size();

    point_buffer.x = 0.5*(pos->pose.at(L - 1).position.x + pos->pose.at(L - 2).position.x);
    point_buffer.y = 0.5*(pos->pose.at(L - 1).position.y + pos->pose.at(L - 2).position.y);
    point_buffer.z = 0.5*(pos->pose.at(L - 1).position.z + pos->pose.at(L - 2).position.z);

    if (robot_trail.points.size() < 1000)
    {
        robot_trail.points.push_back(point_buffer);
    }
    else
    {
        robot_trail.points.erase(robot_trail.points.begin());
        robot_trail.points.push_back(point_buffer);
    }

}

int main( int argc, char** argv)
{
    ros::init(argc, argv, "trail_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Subscriber traj_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, update_line);
    ros::Rate r(30);

    //Define points message
    robot_trail.header.frame_id = "/base_link";
    robot_trail.header.stamp = ros::Time::now();
    robot_trail.ns = "points_and_lines";
    robot_trail.action = visualization_msgs::Marker::ADD;
    robot_trail.pose.orientation.w = 1.0;

    robot_trail.id = 1;

    robot_trail.type = visualization_msgs::Marker::LINE_STRIP;


    //Define the width of robot trail
    robot_trail.scale.x = 0.005;

    // Robot trails are white
    robot_trail.color.r = 1.0f;
    robot_trail.color.g = 1.0f;
    robot_trail.color.b = 1.0f;
    robot_trail.color.a = 1.0;

    while (ros::ok())
    {
        marker_pub.publish(robot_trail);
        ros::spinOnce();

        r.sleep();
    }
}