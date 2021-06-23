#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def callback(msg): #Callback for the subscriber "sub"
    print 'Incoming message: ', msg.data #Print the message


def listener():

    rospy.init_node('listener', anonymous=True) #Initialise the node handle for the node "listener".

    sub = rospy.Subscriber("chatter", Float64MultiArray, callback) #Initialise the subscriber subscribing to the topic "/chatter".

    rospy.spin()


if __name__ == '__main__':
    listener()
