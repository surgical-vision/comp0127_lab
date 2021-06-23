#!/usr/bin/env python

import rospy
import numpy as np
from example_msg_srv.msg import test_msg

def subscribeCallback(msg):

    I = np.identity(3)
    theta = np.sqrt(np.power(msg.rotx.data, 2) + np.power(msg.roty.data, 2) + np.power(msg.rotz.data, 2))
    omega_x = msg.rotx.data/theta
    omega_y = msg.roty.data/theta
    omega_z = msg.rotz.data/theta

    K = np.zeros((3, 3))
    K[0, 1] = -omega_z
    K[1, 0] = omega_z
    K[0, 2] = omega_y
    K[2, 0] = -omega_y
    K[1, 2] = -omega_x
    K[2, 1] = omega_x

    rot_mat = I + np.sin(theta) * K + (1 - np.cos(theta)) * (K.dot(K))

    print 'The incoming matrix is: '
    print rot_mat


def rot_converter():

    rospy.init_node('rotation_subscriber', anonymous=True)
    sub = rospy.Subscriber('publish_rotation', test_msg, subscribeCallback)
    rate = rospy.Rate(10)

    rospy.spin()
    rate.sleep()

if __name__ == '__main__':
    try:
        rot_converter()
    except rospy.ROSInterruptException:
        pass