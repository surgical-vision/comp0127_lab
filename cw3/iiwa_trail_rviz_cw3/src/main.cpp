#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <Eigen/Dense>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

visualization_msgs::Marker reached_points, robot_trail, points;
geometry_msgs::Point point_buffer;
double d;
void update_line(const gazebo_msgs::LinkStates::ConstPtr &pos)
{
    int L = pos->pose.size();

    point_buffer.x = 0.5*(pos->pose.at(L - 1).position.x + pos->pose.at(L - 2).position.x);
    point_buffer.y = 0.5*(pos->pose.at(L - 1).position.y + pos->pose.at(L - 2).position.y);
    point_buffer.z = 0.5*(pos->pose.at(L - 1).position.z + pos->pose.at(L - 2).position.z);

    if (robot_trail.points.size() < 400)
    {
        robot_trail.points.push_back(point_buffer);
    }
    else
    {
        robot_trail.points.erase(robot_trail.points.begin());
        robot_trail.points.push_back(point_buffer);
    }

}

void update_point()
{

    double th = 0.03;
    for (int i = 0; i < robot_trail.points.size(); i++)
    {
        for (int j = 0; j < points.points.size(); j++)
        {
            d = sqrt(pow(robot_trail.points.at(i).x - points.points.at(j).x, 2) +
                     pow(robot_trail.points.at(i).y - points.points.at(j).y, 2) +
                     pow(robot_trail.points.at(i).z - points.points.at(j).z, 2));
            if (d < th)
            {
                reached_points.points.push_back(points.points.at(j));
                points.points.erase(points.points.begin() + j);
                return;
            }
        }
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "trail_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Subscriber traj_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, update_line);
    ros::Rate r(20);

    //Define points message
    points.header.frame_id = robot_trail.header.frame_id = reached_points.header.frame_id = "/world";
    points.header.stamp = robot_trail.header.stamp = reached_points.header.stamp = ros::Time::now();
    points.ns = robot_trail.ns = reached_points.ns = "points_and_lines";
    points.action = robot_trail.action = reached_points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = robot_trail.pose.orientation.w = reached_points.pose.orientation.w = 1.0;

    points.id = 0;
    robot_trail.id = 1;
    reached_points.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    reached_points.type = visualization_msgs::Marker::POINTS;
    robot_trail.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.01;
    points.scale.y = 0.01;

    // Unreached points are red
    points.color.r = 1.0f;
    points.color.a = 1.0;


    // POINTS markers use x and y scale for width/height respectively
    reached_points.scale.x = 0.01;
    reached_points.scale.y = 0.01;

    // Reached points are red
    reached_points.color.g = 1.0f;
    reached_points.color.a = 1.0;


    //Define the width of robot trail
    robot_trail.scale.x = 0.005;

    // Robot trails are white
    robot_trail.color.r = 1.0f;
    robot_trail.color.g = 1.0f;
    robot_trail.color.b = 1.0f;
    robot_trail.color.a = 1.0;

    rosbag::Bag bag;

    bag.open(MY_BAG_PATH1, rosbag::bagmode::Read);

    std::vector<std::string> topics;

    topics.push_back(std::string("target_pose"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    Eigen::Matrix4d handeye_gt;

    handeye_gt << 1, 0, 0, 0.3,
                  0, 1, 0, 0.15,
                  0, 0, 1, 0,
                  0, 0, 0, 1;

    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::TransformStamped::ConstPtr s = m.instantiate<geometry_msgs::TransformStamped>();
        if (s != NULL)
        {
            Eigen::Vector4d p_h;
            geometry_msgs::Point p;
            p_h(0) = s->transform.translation.x;
            p_h(1) = s->transform.translation.y;
            p_h(2) = s->transform.translation.z;
            p_h(3) = 1.0;

            p_h = handeye_gt.inverse() * p_h;

            p.x = p_h(0);
            p.y = p_h(1);
            p.z = p_h(2);

            points.points.push_back(p);
        }

    }


    bag.close();

    while (ros::ok())
    {
        marker_pub.publish(points);
        marker_pub.publish(robot_trail);
        marker_pub.publish(reached_points);

        //update_point();

        ros::spinOnce();

        r.sleep();
    }
}
