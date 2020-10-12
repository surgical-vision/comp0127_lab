#!/usr/bin/env python

import rospy
import math
##TODO: include the library for a msg typed VectorStamped

def talker():

    ##TODO: Define a publisher
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    radius = 1.0
    period = 5.0

    ##TODO: Define your message

    starting_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():

        ##TODO: Input the data into your message. Your message should be a position of a unit circle on the XZ-plane.
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass