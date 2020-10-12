#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def talker():

    pub = rospy.Publisher('chatter', Float64MultiArray, queue_size=10) #Initialise a publisher to the topic "/chatter" with a queue size of 10
    rospy.init_node('talker', anonymous=True) #Initialise the node handle for the node "talker".
    rate = rospy.Rate(10) #ROS shuffling rate = 10hz.
    count = 0

    num_msg = Float64MultiArray() #Initialise the message (type = Float64MultiArray)
    num_msg.data = []

    while not rospy.is_shutdown():

        num_msg.data.append(count) #Add the variable "count" to the list

        pub.publish(num_msg) #Publish the data
        rate.sleep() 
        count += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
