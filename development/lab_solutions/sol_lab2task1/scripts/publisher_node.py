#!/usr/bin/env python

import rospy
import random
##TODO: include the library for a msg for an integer
from std_msgs.msg import Int64

def talker():
    ##TODO: Define a publisher for an integer
    pub = rospy.Publisher('chatter',Int64,queue_size=1000)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count = 0
    while not rospy.is_shutdown():
        ##TODO: Create a message and publish it. Your message should be a random number between 0 and the variable 'count'
        data=random.randint(0,count)
        pub.publish(data)
        
        rate.sleep()
        count += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
