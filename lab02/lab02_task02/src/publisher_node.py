#!/usr/bin/env python

import rospy
import math
import random
##TODO: include the library for a msg typed VectorStamped

def talker():

    ##TODO: Define a publisher, initialize the node, and define a rate


    ##TODO: Define the unit circle radius, and the message to be published


    while not rospy.is_shutdown():
	
	#The VectorStamped type needs a timestamp. This timestamp can be the current time.
        time_stamp=rospy.Time.now()

	##TODO: Define the position (x,y,z) on the unit circle
	

	##TODO: Input the data into your message. 


	##TODO: Publish the message


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
