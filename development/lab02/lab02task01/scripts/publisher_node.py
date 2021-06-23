#!/usr/bin/env python

import rospy
##TODO: include the library for a msg for an integer

def talker():
    ##TODO: Define a publisher for an integer
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count = 0
    while not rospy.is_shutdown():
        ##TODO: Create a message and publish it. Your message should be a random number between 0 and the variable 'count'
        rate.sleep()
        count += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass