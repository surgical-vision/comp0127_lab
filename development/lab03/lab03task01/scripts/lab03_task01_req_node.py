#!/usr/bin/env python

import rospy
import random
##TODO: Include the header file rotate_point service (look in the package example_msg_srv)


def rotate_point_client():

    ##TODO: Wait for rotate_point service (look in the package example_msg_srv)

    while not rospy.is_shutdown():
        ##TODO: Initialise your client
        ##TODO: Generate a random point and a random quaternion (make sure its norm is 1)

        ##TODO: Compute the computational time between requests (from sending a request and getting a response).

        print 'The input point'
        ##TODO: Print out the input and the output points
        print 'The output point'


if __name__ == "__main__":
    try:
        rotate_point_client()
    except rospy.ROSInterruptException:
        pass
