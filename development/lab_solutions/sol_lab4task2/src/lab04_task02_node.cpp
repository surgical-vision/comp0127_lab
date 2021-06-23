#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_eigen/tf2_eigen.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "open_tf_listener_node");

    ros::NodeHandle nh;

    ros::Rate rate(10);

    //TODO: Initialise the tf listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    //TODO: Initialise the tf broadcaster
    tf2_ros::TransformBroadcaster br;

    while (nh.ok())
    {
        //TODO: Create a routine to get the transformation from "world" to "link5"
        geometry_msgs::TransformStamped T;

        try
        {
            T = tfBuffer.lookupTransform("world", "link5", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        //TODO: Apply the transformation such that the new frame is located at the point 0.05m away from the end-effector (pointing-out direction)
        //and has the same orientation.
    	T.header.frame_id = "link5";
    	T.child_frame_id = "added_frame";
    	T.transform.translation.x = 0.12;//link 5 to the end effector is 0.07, and another 0.05 to the added frame
  	    T.transform.translation.y = 0.0;
  	    T.transform.translation.z = 0.0;
  	    tf2::Quaternion q;
        q.setRPY(0, 0, 0);
       	T.transform.rotation.x = q.x();
       	T.transform.rotation.y = q.y();
       	T.transform.rotation.z = q.z();
  		T.transform.rotation.w = q.w();
        //TODO: Broadcast the frame
		br.sendTransform(T);
		
        rate.sleep();
    }
    return 52;
}