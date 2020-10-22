#!/usr/bin/env python
import rospy
##TODO: include the library for a msg for an integer
from std_msgs.msg import Int64

##TODO: complete the callback function. Your callback function should print out the incoming message.

def callback(data):
    print 'Incoming number between 0 and the var count: ',str(data)

def listener():

    rospy.init_node('listener', anonymous=True)

    ##TODO: Define a subscriber.
    rospy.Subscriber("chatter", Int64, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()