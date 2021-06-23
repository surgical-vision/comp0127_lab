#!/usr/bin/env python

import rospy
##TODO: include the library for a msg for an array integer

def talker():

    ##TODO: Define a nodehandle and a publisher
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count = 0

    ##TODO: Define the message

    while not rospy.is_shutdown():

        ##TODO: For every loop, keep appending the array with the variable 'count' (If count = 4, an output array should be [1, 2, 3, 4])
        ##TODO: publish the message
        rate.sleep()
        count += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass