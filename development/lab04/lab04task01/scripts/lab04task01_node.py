#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros

##TODO: Fill in the DH parameters
a = [0.0, 0.0, 0.0, 0.0]
alpha = [0.0, 0.0, 0.0, 0.0]
d = [0.0, 0.0, 0.0, 0.0]
theta = [0.0, 0.0, 0.0, 0.0]

name_link = ['fkine_link_1', 'fkine_link_2', 'fkine_link_3', 'fkine_link_4']

##TODO: Initialise the broadcaster

##TODO: Fill in this function
def fkine_standard(a, alpha, d, theta):
    A = np.zeros((4, 4))
    return A


def rotmat2q(T):
    q = Quaternion()

    angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1)/2)

    xr = T[2, 1] - T[1, 2]
    yr = T[0, 2] - T[2, 0]
    zr = T[1, 0] - T[0, 1]

    x = xr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    y = yr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    z = zr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))

    q.w = np.cos(angle/2)
    q.x = x * np.sin(angle/2)
    q.y = y * np.sin(angle/2)
    q.z = z * np.sin(angle/2)

    return q


##TODO: Fill in this callback function
def fkine(joint_msg):
    print 'test'


def fkine_main():
    rospy.init_node('open_fkine_node')

    ##TODO: Initialise your subscriber
    rate = rospy.Rate(10)
    rospy.spin()
    rate.sleep()


if __name__ == '__main__':
    try:
        fkine_main()
    except rospy.ROSInterruptException:
        pass