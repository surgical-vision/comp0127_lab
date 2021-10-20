#! /usr/bin/env python

import rospy
import numpy as np
import random
import time

#TODO: import the SRV file from its corresponding folder, as well as its Request


def point_rotation_client():
    rospy.wait_for_service('rotate_pt')

    while not rospy.is_shutdown():
        #TODO: Initialise the ROS service client. It takes two arguments: The name of the service, and the service definition.
        client = rospy.ServiceProxy('rotate_pt', point_rot)
        
        req = point_rotRequest()

        #TODO: create a random request point, and a random request quaternion
        


	res = client(req)
        print(res)

        time.sleep(3)


if __name__ == '__main__':
    try:
        point_rotation_client()
    except rospy.ROSInterruptException:
        pass
