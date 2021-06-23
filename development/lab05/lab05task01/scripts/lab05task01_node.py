#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
from sensor_msgs.msg import JointState

a = [0.0, 0.265699, 0.03, 0.0, 0.0, 0.0]
alpha = [-pi/2, 0.0, -pi/2, -pi/2, -pi/2, 0.0]
d = [0.159, 0.0, 0.0, 0.258, 0.0, -0.123]
theta = [0.0, -pi/2+np.arctan(0.03/0.264), -np.arctan(0.03/0.264), 0.0, 0.0, 0.0]


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


def fkine(joint_msg):

    T = np.identity(4)
    ##TODO: fill in the forward kinematic routine
    ##TODO: Compute the Jacobian matrix and print it


def fkine_main():
    rospy.init_node('robotis_jacob_node')

    sub = rospy.Subscriber('/joint_states', JointState, fkine)
    rate = rospy.Rate(10)
    rospy.spin()
    rate.sleep()


if __name__ == '__main__':
    try:
        fkine_main()
    except rospy.ROSInterruptException:
        pass
