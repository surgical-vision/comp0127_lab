#include "ros/ros.h"
//TODO: include the library for a msg for an integer
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    //TODO: Define a nodehandle and a publisher.
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/chatter", 1000);

    ros::Rate loop_rate(10);

    std_msgs::Float64 msg;
    msg.data = 1;
    int count = 0;

    while (ros::ok())
    {
	
	//TODO: Create a message and publish it. Your message should be a random number between 0 and the variable 'count'        
	if (count == 0){
		msg.data = 0;
	}
	else{	
		msg.data = rand() % count;
	}

        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }


    return 0;
}

