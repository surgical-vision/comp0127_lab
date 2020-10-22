#!/usr/bin/env python
import rospy
import numpy as np
##TODO: include the library for a msg for an array integer
from std_msgs.msg import Int64MultiArray

##TODO: Complete the callback function. Your callback function should compute the sum (1 + 1/2 + 1/3 +... + 1/n) and print the output on the screen

def callback(msg):
	 
	my_array=msg.data
	
	sum=0
	for i in range(len(my_array)):
		if (my_array[i] != 0):
			sum+=(1/float(my_array[i]))
	print str(sum)

def listener():

    rospy.init_node('listener', anonymous=True)

    ##TODO: Define your subscriber and link this with the callback function.
    rospy.Subscriber("chatter", Int64MultiArray, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()