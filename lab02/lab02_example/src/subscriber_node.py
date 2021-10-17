#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def callback(msg):
	print 'Incoming message: ', msg.data

def listener():
	rospy.init_node('subscriber', anonymous = True)
	sub = rospy.Subscriber('chatter', Float64MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
