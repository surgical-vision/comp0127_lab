#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
from numpy import arctan2
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import pdb

##TODO: Fill in the DH parameters
a = [0.0, 0.13, 0.124, 0.07]
alpha = [-pi/2, 0.0, 0.0, 0.0]
d = [0.075, 0.0, 0.0, 0.0]
theta = [0.0, -pi/2 + arctan2(0.024, 0.128), arctan2(0.128, 0.024), 0.0]

name_link = ['fkine_link_1', 'fkine_link_2', 'fkine_link_3', 'fkine_link_4']

##TODO: Initialise the broadcaster
br = tf2_ros.TransformBroadcaster()

##TODO: Fill in this function
def fkine_standard(a, alpha, d, theta):
    A = np.zeros((4, 4))

    A[0, 0] = np.cos(theta)
    A[0, 1] = -np.sin(theta)*np.cos(alpha)
    A[0, 2] = np.sin(theta)*np.sin(alpha)
    A[0, 3] = a*np.cos(theta)

    A[1, 0] = np.sin(theta)
    A[1, 1] = np.cos(theta)*np.cos(alpha)
    A[1, 2] = -np.cos(theta)*np.sin(alpha)
    A[1, 3] = a*np.sin(theta)

    A[2, 1] = np.sin(alpha)
    A[2, 2] = np.cos(alpha)
    A[2, 3] = d

    A[3, 3] = 1.0

    return A


def rotmat2q(T):
    q = Quaternion()

    angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1)/2)

    if (angle == 0):
        q.w = 1
        q.x = 0
        q.y = 0
        q.z = 0

    else:
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
    T = np.identity(4)

    transform = TransformStamped()
    for i in range(4):

        A = fkine_standard(a[i], alpha[i], d[i], theta[i] + joint_msg.position[i])

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'world'
        transform.child_frame_id = name_link[i]

        T = T.dot(A)


        transform.transform.translation.x = T[0, 3]
        transform.transform.translation.y = T[1, 3]
        transform.transform.translation.z = T[2, 3]
        transform.transform.rotation = rotmat2q(T)

        br.sendTransform(transform)


def fkine_main():
    rospy.init_node('open_fkine_node')

    ##TODO: Initialise your subscriber
    sub = rospy.Subscriber('/joint_states', JointState, fkine)
    rate = rospy.Rate(10)
    rospy.spin()
    rate.sleep()


if __name__ == '__main__':
    try:
        fkine_main()
    except rospy.ROSInterruptException:
        pass