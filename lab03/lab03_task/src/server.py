#!/usr/bin/env python

import rospy
import numpy as np

# TODO: Include all the required service classes


def convert_rotmat2quat(request):
    """Callback ROS service function to convert quaternion to Euler z-y-x representation

    Args:
        request (rotmat2quatRequest): lab03_task service message, containing
        the rotation matrix you need to convert.

    Returns:
        rotmat2quatResponse: lab03_task service response, in which 
        you store the requested quaternion
    """

    m = np.array(request.R.data).reshape(3,3)
    tr = np.trace(m)
    # TODO complete the function to transform a rotation matrix to quaternion


    response = rotmat2quatResponse()
    return response

def rotation_converter():
    rospy.init_node('rotation_converter')

    # TODO: Initialise the service


    rospy.spin()

if __name__ == "__main__":
    rotation_converter()
