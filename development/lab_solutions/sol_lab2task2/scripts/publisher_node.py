#!/usr/bin/env python

import rospy
import numpy as np
##TODO: include the library for a msg for an array integer
from std_msgs.msg import Int64MultiArray

def talker():

    ##TODO: Define a publisher
    pub = rospy.Publisher('chatter',Int64MultiArray,queue_size=1000)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    

    count = 0
    my_array=[] 
    while not rospy.is_shutdown():

        ##TODO: For every loop, keep appending the array with the variable 'count' (If count = 4, an output array should be [1, 2, 3, 4])
        my_array=np.append(my_array,count)
        array_for_publish=Int64MultiArray(data=my_array)
        ##TODO: publish the message
        pub.publish(array_for_publish);

        rate.sleep()
        count += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass