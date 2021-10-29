#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
from numpy import arctan2
from geometry_msgs.msg import Quaternion
#import the message type that holds data describing robot joint angles
from sensor_msgs.msg import JointState
#import the class that publishes coordinate frame transform information
from tf2_ros import TransformBroadcaster
#import the message type that expresses a transformt from one coordinate frame to another
from geometry_msgs.msg import TransformStamped, Quaternion

#The OpenManipulator DH parameters
#a1 is 0.012 rather than 0 to align with the URDF file
a = [0.012, 0.13, 0.124, 0.07]
alpha = [pi/2, 0.0, 0.0, 0.0]
d = [0.075, 0.0, 0.0, 0.0]
theta = [0.0, -1.385, 1.385, 0.0]

#TODO: #define the frame names
name_link = ['fkine_link_1']

def forward_kinematics(a1, alpha1, d1, theta1):
    #This function returns the hard-coded T01 matrix for joint frame 1. 
    #In your coursework you should compute the forward kinematics for every joint frame.
    #Hard-coded versions of matrices are generally bad practice and here is for demonstration purposes only.

    T01 = np.zeros((4, 4))

    T01[0, 0] = np.cos(theta1)
    T01[0, 1] = -np.sin(theta1)*np.cos(alpha1)
    T01[0, 2] = np.sin(theta1)*np.sin(alpha1)
    T01[0, 3] = a1*np.cos(theta1)

    T01[1, 0] = np.sin(theta1)
    T01[1, 1] = np.cos(theta1)*np.cos(alpha1)
    T01[1, 2] = -np.cos(theta1)*np.sin(alpha1)
    T01[1, 3] = a1*np.sin(theta1)

    T01[2, 1] = np.sin(alpha1)
    T01[2, 2] = np.cos(alpha1)
    T01[2, 3] = d1

    T01[3, 3] = 1.0

    return T01

def rotmat2q(T):
    #function that transforms a rotation matrix to quaternion representation
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


def fkine_wrapper(joint_msg, br):
    ##TODO: Fill in this callback function
    transform = TransformStamped()

    #This loop should iterate through all joints. In this simplified example, we are only using joint frame 1.
    for i in range(1):
        
        #for every joint, calculate the forward kinematics -> here hard-coded
        T01 = forward_kinematics(a[0], alpha[0], d[0], theta[0] + joint_msg.position[i])

        #Here we define the transform from joint frame 0 to joint frame 1 using the tf2 broadcaster.
        #define the transfrom timestamp
        transform.header.stamp = rospy.Time.now()
        #define the transfrom header frame (parent frame)
        transform.header.frame_id = 'world'
        #define the transfrom child frame
        transform.child_frame_id = name_link[i]
        #populate the transform field. It consists of translation and rotation.
        transform.transform.translation.x = T01[0, 3]
        transform.transform.translation.y = T01[1, 3]
        transform.transform.translation.z = T01[2, 3]
        transform.transform.rotation = rotmat2q(T01)

        #broadcast the transform to tf2
        br.sendTransform(transform)

def main():
    rospy.init_node('open_forward_kinematic_node')

    ##TODO: Initialise the broadcaster
    br = TransformBroadcaster()

    ##TODO: Initialise the subscriber.
    sub = rospy.Subscriber('/joint_states', JointState, fkine_wrapper, br)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
