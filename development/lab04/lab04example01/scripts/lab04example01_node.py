#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros

#Initialise all DH parameters
a = [0.0, 0.265699, 0.03, 0.0, 0.0, 0.0]
alpha = [-pi/2, 0.0, -pi/2, -pi/2, -pi/2, 0.0]
d = [0.159, 0.0, 0.0, 0.258, 0.0, -0.123]
theta = [0.0, -pi/2+np.arctan(0.03/0.264), -np.arctan(0.03/0.264), 0.0, 0.0, 0.0]

name_link = ['fkine_link_1', 'fkine_link_2', 'fkine_link_3', 'fkine_link_4', 'fkine_link_5', 'fkine_link_6']

##Declaring a transformBroadcaster like this is not a very optimal way.
##The very optimal way is to create a "class" for it.
##We will ask you to do this in the next Coursework. :)

br = tf2_ros.TransformBroadcaster()

def fkine_standard(a, alpha, d, theta):
#Define a transformation in standard DH.
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
#Define a function  for converting rotation matrix to a quaternion
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


def fkine(joint_msg):
#Forward kinematic
    T = np.identity(4)

    transform = TransformStamped()
    for i in range(6):

        if (i == 4) or (i == 5):
            A = fkine_standard(a[i], alpha[i], d[i], theta[i] - joint_msg.position[i])
        else:
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
    rospy.init_node('robotis_fkine_node')

    sub = rospy.Subscriber('/joint_states', JointState, fkine)
    rate = rospy.Rate(10)
    rospy.spin()
    rate.sleep()


if __name__ == '__main__':
    try:
        fkine_main()
    except rospy.ROSInterruptException:
        pass
