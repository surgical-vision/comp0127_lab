#! /usr/bin/env python

import rospy
import numpy as np

#TODO: import the SRV file from its corresponding folder, as well as its Response
from lab03_example_srv.srv import point_rot
from lab03_example_srv.srv import point_rotResponse

def handle_point_rotation(req):
    #Function that is appllied to the given request message

    #TODO: After temporarily storing all request values into variables, convert the requested point by the requested quaternion and return it as a response.
    px = req.p.x
    py = req.p.y
    pz = req.p.z

    qx = req.q.x
    qy = req.q.y
    qz = req.q.z
    qw = req.q.w

    qxs = np.power(qx, 2)
    qys = np.power(qy, 2)
    qzs = np.power(qz, 2)

    res = point_rotResponse()

    res.out_p.x = px*(1 - 2*qys - 2*qzs) + py*(2*(qx*qy - qz*qw)) + pz*(2*(qx*qz - qy*qw))
    res.out_p.y = px * (2*(qx*qy - qz*qw)) + py * (1 - 2*qxs - 2*qzs) + pz * (2*(qy*qz - qx*qw))
    res.out_p.z = px * (2*(qx*qz - qy*qw)) + py * (2*(qy*qz - qx*qw)) + pz * (1 - 2*qys - 2*qxs)

    return res

def rotate_point_service():
    #TODO: Initiatlise the 'rotate_point' node
    rospy.init_node('rotate_point', anonymous=True)
    #TODO: Create a ROS service with three arguments: the ROS service name, the defined service from the srv file, and the function to be applied on the given request message
    s = rospy.Service('rotate_pt', point_rot, handle_point_rotation)
    #TODO: Initiatlise the 'rotate_point' node
    rospy.spin()

if __name__ == '__main__':
    rotate_point_service()
