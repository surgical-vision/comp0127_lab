#! /usr/bin/env python

import rospy
import numpy as np

#TODO: import the SRV file from its corresponding folder, as well as its Response


def handle_point_rotation(req):
    #Function that is appllied to the given request message

    #TODO: After temporarily storing all request values into variables, convert the requested point by the requested quaternion and return it as a response.
    

    return res

def rotate_point_service():
    #TODO: Initiatlise the 'rotate_point' node
    
    #TODO: Create a ROS service with three arguments: the ROS service name, the defined service from the srv file, and the function to be applied on the given request message


    rospy.spin()

if __name__ == '__main__':
    rotate_point_service()
