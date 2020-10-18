#!/usr/bin/env python

import rospy
import random
from example_msg_srv.srv import test_srvRequest
from example_msg_srv.srv import test_srv


def rot_convert_client():

    rospy.wait_for_service('rot_convert') #Wait for the service "rot_convert" to become available.

    while not rospy.is_shutdown():
        client = rospy.ServiceProxy('rot_convert', test_srv) #Initialise client for the service "rot_convert"

        req = test_srvRequest() #Initialise request message (defined by test_srvRequest).
        req.x.data = random.uniform(-2.0, 2.0)
        req.y.data = random.uniform(-1.7, 1.7)
        req.z.data = random.uniform(-1.0, 1.8)

        resp = client(req) #Get the response from the service.

        print 'The resulting quaternion'
        print resp.q


if __name__ == "__main__":
    try:
        rot_convert_client()
    except rospy.ROSInterruptException:
        pass
