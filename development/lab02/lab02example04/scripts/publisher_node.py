#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PointStamped

def talker():

    pub = rospy.Publisher('chatter', PointStamped, queue_size=10) #Initialise publisher
    rospy.init_node('talker', anonymous=True) #Initialise the node handle
    rate = rospy.Rate(10) 

    radius = 5
    period = 5

    msg = PointStamped()

    starting_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():

        msg.header.stamp = rospy.Time.now() #Input the current time for the timestamp.

        msg.point.x = radius * math.cos(2 * math.pi * (msg.header.stamp.to_sec() - starting_time)/period) 
        msg.point.y = radius * math.sin(2 * math.pi * (msg.header.stamp.to_sec() - starting_time)/period)
        msg.point.z = 0.0

        pub.publish(msg) #Publish the position of a point running on a 5 unit radius-circle
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
