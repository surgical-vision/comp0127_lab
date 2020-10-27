#!/usr/bin/env python

import rospy
import random
import timeit
import numpy as np
##TODO: Include the header file rotate_point service (look in the package example_msg_srv)
from sol_example_msg_srv.srv import rotate_pointRequest
from sol_example_msg_srv.srv import rotate_point

import pdb

def rotate_point_client():

    ##TODO: Wait for rotate_point service (look in the package example_msg_srv)
    rospy.wait_for_service('rotate_point')

    while not rospy.is_shutdown():
        ##TODO: Initialise your client
        client = rospy.ServiceProxy('rotate_point', rotate_point)
        ##TODO: Generate a random point and a random quaternion (make sure its norm is 1)

        req = rotate_pointRequest()
        quat = np.random.rand(4)
        quat_unit = quat/np.linalg.norm(quat)
        req.q.x = quat_unit[0]
        req.q.y = quat_unit[1]
        req.q.z = quat_unit[2]
        req.q.w = quat_unit[3]
        req.input_p.x = random.uniform(-2.0, 2.0)
        req.input_p.y = random.uniform(-1.7, 1.7)
        req.input_p.z = random.uniform(-1.0, 1.8)

        ##TODO: Compute the computational time between requests (from sending a request and getting a response).
        rospy.init_node('gaze', anonymous=False)
        start = rospy.get_time()
        resp = client(req)
        end = rospy.get_time()
        dt=end - start
        print 'time elapsed: '+ str(dt)

        print 'The input point'
        print req.input_p.x 
        print req.input_p.y
        print req.input_p.z

        print 'The output point'
        print resp.output_p.x 
        print resp.output_p.y
        print resp.output_p.z
        


if __name__ == "__main__":
    try:
        rotate_point_client()
    except rospy.ROSInterruptException:
        pass
