#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion
import pdb

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


if __name__ == '__main__':
    rospy.init_node('open_tf_listener_node')

    ##TODO: Initialise your tf listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    ##TODO: Initialise your tf broadcaster
    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        ##TODO: Create a routine to get the transformation from "world" to "link5"
        try:
            T = tfBuffer.lookup_transform('world', 'link5', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
            rate.sleep()
            continue

        ##TODO: Apply the transformation such that the new frame is located at the point 0.05 m away from the end-effector (pointing-out direction)
        ## and has the same orientation.
        Toffset = np.identity(4)
        Toffset[0,3] = 0.07 + 0.05

        Tnew = TransformStamped()
        Tnew.header.stamp = rospy.Time.now()
        Tnew.header.frame_id = 'link5'
        Tnew.child_frame_id = 'added_frame'
        Tnew.transform.translation.x = Toffset[0, 3]
        Tnew.transform.translation.y = Toffset[1, 3]
        Tnew.transform.translation.z = Toffset[2, 3]
        Tnew.transform.rotation = rotmat2q(Toffset)

        ##TODO: Broadcast the frame
        br.sendTransform(Tnew)

        rate.sleep()