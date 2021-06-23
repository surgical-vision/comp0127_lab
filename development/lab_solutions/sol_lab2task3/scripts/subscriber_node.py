#!/usr/bin/env python
import rospy
import random
##TODO: include the library for a msg typed VectorStamped
from geometry_msgs.msg import Vector3Stamped

##TODO: complete your callback function. Your callback function should add the noisy numbers to the incoming x and z positions and print.
def callback(msg, args):
    x_noisy_pose = msg.vector.x + args[0];
    z_noisy_pose = msg.vector.z - args[1];
    print 'The current point is '
    print str(x_noisy_pose)
    print str(msg.vector.y)
    print str(z_noisy_pose)

def listener():

    rospy.init_node('listener', anonymous=True)

    ##TODO: Create two random numbers between 0 and 1.
    input1=random.randint(0,1)
    input2=random.randint(0,1)
    ##TODO: Create a subscriber and links to the callback function.
    rospy.Subscriber("chatter", Vector3Stamped, callback, (input1, input2))

    #rate = rospy.Rate(100000)
    rospy.spin()
    #rate.sleep()

if __name__ == '__main__':
    listener()
