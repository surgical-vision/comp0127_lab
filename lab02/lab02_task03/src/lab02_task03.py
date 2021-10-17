#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64


def joint_pub():
    rospy.init_node('trajectory_generator', anonymous=True)
    rate = rospy.Rate(10)

    ##TODO: Define each joint publisher, by identifying the appropriate publishing topic.
    pub1 = rospy.Publisher('/EffortJointInterface_J1_controller/command', Float64, queue_size=100)
    pub2 = rospy.Publisher('/EffortJointInterface_J2_controller/command', Float64, queue_size=100)
    pub3 = rospy.Publisher('/EffortJointInterface_J3_controller/command', Float64, queue_size=100)
    pub4 = rospy.Publisher('/EffortJointInterface_J4_controller/command', Float64, queue_size=100)

    ##TODO: Define the messages to be published.
    joint1 = Float64()
    joint2 = Float64()
    joint3 = Float64()
    joint4 = Float64()

    while not rospy.is_shutdown():

        t = rospy.Time.now().secs
	
	##TODO: Define the joint trajectories.
        joint1.data = 200 * math.pi / 180 * math.sin(2 * math.pi * t / 10)
        joint2.data =  50 * math.pi / 180 * math.sin(2 * math.pi * t / 12)
        joint3.data = -80 * math.pi / 180 * math.sin(2 * math.pi * t / 15)
        joint4.data =  60 * math.pi / 180 * math.sin(2 * math.pi * t / 11)

	##TODO: Publish all messages.
        pub1.publish(joint1)
        pub2.publish(joint2)
        pub3.publish(joint3)
        pub4.publish(joint4)

        rate.sleep()


if __name__ == '__main__':
    try:
        joint_pub()
    except rospy.ROSInterruptException:
        pass
