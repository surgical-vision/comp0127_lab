#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Transform.h>
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

    if (robot_trail.points.size() < 800)
    {
        robot_trail.points.push_back(point_buffer);
    }
    else
    {
        robot_trail.points.erase(robot_trail.points.begin());
        robot_trail.points.push_back(point_buffer);
    }

}

void update_point(int data)
{

	double th = 0.02;
	if (data > 2)
		th = 0.06;
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

Eigen::Matrix4d dh_matrix_standard(double a, double alpha, double d, double theta)
{
    Eigen::Matrix4d A;
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

Eigen::Matrix4d youbot_forward_kine(double joint[])
{

    double a[5] = {-0.033, 0.155, 0.135, -0.002, 0.0};
    double alpha[5] = {M_PI_2, 0.0, 0.0, M_PI_2, M_PI};
    double d[5] = {0.145, 0.0, 0.0, 0.0, -0.185};
    double theta[5] = {170*M_PI/180 - M_PI, M_PI_2 - 65*M_PI/180, 146*M_PI/180, -M_PI_2-102.5*M_PI/180, M_PI - 167.5*M_PI/180};

    Eigen::Matrix4d A = Eigen::Matrix4d::Identity(4, 4);

    for (int i = 0; i < 5;i++)
    {
        if (i == 0)
            A = A * dh_matrix_standard(a[i], alpha[i], d[i], theta[i] - joint[i]);
        else
            A = A * dh_matrix_standard(a[i], alpha[i], d[i], theta[i] + joint[i]);
    }

    return A;

}



int main( int argc, char** argv )
{
    ros::init(argc, argv, "trail_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Subscriber traj_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, update_line);
    ros::Rate r(20);

    int checkpoint_data = 1;

    //Define points message
    points.header.frame_id = robot_trail.header.frame_id = reached_points.header.frame_id = "/base_link";
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

    bag.open(MY_BAG_PATH, rosbag::bagmode::Read);

    std::vector<std::string> topics;

    if ((checkpoint_data == 1))
    {
        topics.push_back(std::string("joint_data"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

                foreach(rosbag::MessageInstance const m, view)
                    {
                        sensor_msgs::JointState::ConstPtr s = m.instantiate<sensor_msgs::JointState>();
                        if (s != NULL)
                        {

                            double joint[5];

                            for (int i = 0; i < 5; i++)
	                    	joint[i] = s->position.at(i);


                            Eigen::Matrix4d current_pose = youbot_forward_kine(joint);

                            geometry_msgs::Point p;
                            p.x = current_pose(0, 3);
                            p.y = current_pose(1, 3);
                            p.z = current_pose(2, 3);
                            points.points.push_back(p);

                        }

                    }

    }
    else
    {
        topics.push_back(std::string("target_position"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

                foreach(rosbag::MessageInstance const m, view)
                    {
                        geometry_msgs::Transform::ConstPtr s = m.instantiate<geometry_msgs::Transform>();
                        if (s != NULL)
                        {
                            geometry_msgs::Point p;
                            p.x = s->translation.x;
                            p.y = s->translation.y;
                            p.z = s->translation.z;
                            points.points.push_back(p);
                        }

                    }
    }

    bag.close();

    while (ros::ok())
    {
        marker_pub.publish(points);
        marker_pub.publish(robot_trail);
        marker_pub.publish(reached_points);

        update_point(checkpoint_data);

        ros::spinOnce();

        r.sleep();
    }
}
