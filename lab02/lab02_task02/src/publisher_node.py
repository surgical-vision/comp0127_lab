#!/usr/bin/env python

import rospy
import math
import random
##TODO: include the library for a msg typed VectorStamped
from geometry_msgs.msg import Vector3Stamped

def talker():

    ##TODO: Define a publisher, initialize the node, and define a rate
    pub = rospy.Publisher('chatter',Vector3Stamped,queue_size=1000)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    ##TODO: Define the unit circle radius, and the message to be published
    radius = 1.0
    vector_for_publish=Vector3Stamped()

    while not rospy.is_shutdown():
	
	#The VectorStamped type needs a timestamp. This timestamp can be the current time.
        time_stamp=rospy.Time.now()

	##TODO: Define the position (x,y,z) on the unit circle
	angle = random.uniform(-2*math.pi, 2*math.pi)
	x = radius*math.cos(angle)
	y = 0
	z = radius*math.sin(angle)	

	##TODO: Input the data into your message. 
        vector_for_publish.header.stamp=time_stamp
        vector_for_publish.vector.x=x
        vector_for_publish.vector.y=y
        vector_for_publish.vector.z=z

	##TODO: Publish the message
        pub.publish(vector_for_publish)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
