#!/usr/bin/env python

import rospy
import numpy as np
##TODO: Include the header file rotate_point service (look in the package example_msg_srv)

##TODO: Complete this function
def rotate_point_3d():
    print 'test'


def rotate_point_server():
    rospy.init_node('rotate_point_server')
    ##TODO: Intialise the service 'rotate_point' and have it calls the function 'rotate_point_3d'
    rospy.spin()


if __name__ == "__main__":
    rotate_point_server()