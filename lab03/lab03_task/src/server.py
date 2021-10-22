#!/usr/bin/env python

import rospy
import numpy as np

# TODO: Include all the required service classes
from lab03_task.srv import rotmat2quat, rotmat2quatResponse, rotmat2quatRequest

def convert_rotmat2quat(request):
    """Callback ROS service function to convert quaternion to Euler z-y-x representation

    Args:
        request (rotmat2quatRequest): lab03_task service message, containing
        the rotation matrix you need to convert.

    Returns:
        rotmat2quatResponse: lab03_task service response, in which 
        you store the requested quaternion
    """

    # TODO complete the function to transform a rotation matrix to quaternion
    m = np.array(request.R.data).reshape(3,3)
    tr = np.trace(m)

    response = rotmat2quatResponse()
    theta = np.arccos((tr - 1) / 2)
    if theta == 0:
        response.q.x = 0
        response.q.y = 0
        response.q.z = 0
        response.q.w = 1

    elif theta == np.pi or theta == -np.pi:
        K = 0.5 * (m + np.identity(3))
        response.q.x = np.sin(theta / 2) * np.sqrt(K[0, 0])
        response.q.y = np.sin(theta / 2) * np.sqrt(K[1, 1])
        response.q.z = np.sin(theta / 2) * np.sqrt(K[2, 2])
        response.q.w = 0
  
    else:
        s = 1 / (2 * np.sin(theta))
        rx = s * (m[2, 1] - m[1, 2])
        ry = s * (m[0, 2] - m[2, 0])
        rz = s * (m[1, 0] - m[0, 1])

        response.q.x = rx * np.sin(theta / 2)
        response.q.y = ry * np.sin(theta / 2)
        response.q.z = rz * np.sin(theta / 2)
        response.q.w = np.cos(theta / 2)


    return response

def rotation_converter():
    rospy.init_node('rotation_converter')

    # TODO: Initialise the service
    rospy.Service('rotmat2quat', rotmat2quat, convert_rotmat2quat)

    rospy.spin()

if __name__ == "__main__":
    rotation_converter()
