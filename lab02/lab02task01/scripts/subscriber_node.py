#!/usr/bin/env python
import rospy
##TODO: include the library for a msg for an integer

##TODO: complete the callback function. Your callback function should print out the incoming message.

def callback(data):
    print 'Delete this line.'

def listener():

    rospy.init_node('listener', anonymous=True)

    ##TODO: Define a nodehandle and a subscriber.

    rospy.spin()


if __name__ == '__main__':
    listener()