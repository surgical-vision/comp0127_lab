#!/usr/bin/env python
import rospy
##TODO: include the library for a msg typed VectorStamped

##TODO: complete your callback function. Your callback function should add the noisy numbers to the incoming x and z positions and print.
def callback(msg, args):
    print 'Remove this line'

def listener():

    rospy.init_node('listener', anonymous=True)

    ##TODO: Create two random numbers between 0 and 1.
    ##TODO: Create a subscriber and links to the callback function.

    rate = rospy.Rate(100000)
    ##TODO: Create a subscriber and links to the callback function.

    rospy.spin()
    rate.sleep()

if __name__ == '__main__':
    listener()
