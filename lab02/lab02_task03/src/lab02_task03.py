#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64


def joint_pub():
    rospy.init_node('trajectory_generator', anonymous=True)
    rate = rospy.Rate(10)

    ##TODO: Define each joint publisher, by identifying the appropriate publishing topic.
    pub1 = rospy.Publisher('', Float64, queue_size=100)
    pub2 = rospy.Publisher('', Float64, queue_size=100)
    pub3 = rospy.Publisher('', Float64, queue_size=100)
    pub4 = rospy.Publisher('', Float64, queue_size=100)

    ##TODO: Define the messages to be published.


    while not rospy.is_shutdown():

        t = rospy.Time.now().secs
	
	##TODO: Define the joint trajectories.


	##TODO: Publish all messages.


        rate.sleep()


if __name__ == '__main__':
    try:
        joint_pub()
    except rospy.ROSInterruptException:
        pass
