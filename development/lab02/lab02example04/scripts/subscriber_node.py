#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PointStamped

def callback(msg, args): #Subscriber callback with additional arguments
    x_noisy = msg.point.x + args[0]
    y_noisy = msg.point.y - args[1]
    print 'The current point is (%.3f, %.3f)' % (x_noisy, y_noisy)

def listener():

    rospy.init_node('listener', anonymous=True)

    test_input1 = 10
    test_input2 = -10
    rate = rospy.Rate(100000)
    sub = rospy.Subscriber("chatter", PointStamped, callback, (test_input1, test_input2)) #Initialise a subscriber with additional arguments

    rospy.spin()
    rate.sleep()

if __name__ == '__main__':
    listener()
