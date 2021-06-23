#!/usr/bin/env python
import rospy
##TODO: include the library for a msg for an array integer

##TODO: Complete the callback function. Your callback function should compute the sum (1 + 1/2 + 1/3 +... + 1/n) and print the output on the screen

def callback(msg):
    print 'Delete this line'


def listener():

    rospy.init_node('listener', anonymous=True)

    ##TODO: Define your subscriber and link this with the callback function.

    rospy.spin()


if __name__ == '__main__':
    listener()