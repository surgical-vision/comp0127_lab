#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def talker():
	pub = rospy.Publisher('chatter', Float64MultiArray, queue_size = 100)
	rospy.init_node('publisher', anonymous = True)
	rate = rospy.Rate(1)

	count = 0
	num_msg = Float64MultiArray()
	num_msg.data = []

	while not rospy.is_shutdown():
		num_msg.data.append(count)
		
		pub.publish(num_msg)
		rate.sleep()
		
		count += 1
		print 'Outgoing message: ', num_msg.data


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
