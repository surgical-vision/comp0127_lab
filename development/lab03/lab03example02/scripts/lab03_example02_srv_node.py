#!/usr/bin/env python

import rospy
import numpy as np
from example_msg_srv.srv import test_srvResponse
from example_msg_srv.srv import test_srv


def handle_convert_rot_quat(req): #Callback when the service is called.
    angle = np.sqrt(np.power(req.x.data, 2) + np.power(req.y.data, 2) + np.power(req.z.data, 2))
    x = req.x.data/angle
    y = req.y.data/angle
    z = req.z.data/angle

    res = test_srvResponse()
    res.q.x = x * np.sin(angle / 2)
    res.q.y = y * np.sin(angle / 2)
    res.q.z = z * np.sin(angle / 2)
    res.q.w = np.cos(angle / 2)

    return res



def convert_rot_server():
    rospy.init_node('convert_rot_server') #Initialise the node handle
    s = rospy.Service('rot_convert', test_srv, handle_convert_rot_quat) #Initialise ROS service named "rot_convert" using the service defined by the file test_srv in example_msg_srv.srv
    rospy.spin()


if __name__ == "__main__":
    convert_rot_server()
