#!/usr/bin/env python

import rospy
import math
##TODO: include the library for a msg typed VectorStamped
from geometry_msgs.msg import Vector3Stamped

def talker():

    ##TODO: Define a publisher
    pub = rospy.Publisher('chatter',Vector3Stamped,queue_size=1000)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    radius = 1.0
    period = 5.0

    ##TODO: Define your message

    starting_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():

        ##TODO: Input the data into your message. Your message should be a position of a unit circle on the XZ-plane.
        time_stamp=rospy.Time.now()
        angle=2*math.pi*(time_stamp.to_sec()-starting_time)/period
        vector_for_publish=Vector3Stamped()
        vector_for_publish.header.stamp=time_stamp
        vector_for_publish.vector.x=radius*math.cos(angle)
        vector_for_publish.vector.y=0
        vector_for_publish.vector.z=radius*math.sin(angle)
        pub.publish(vector_for_publish)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
