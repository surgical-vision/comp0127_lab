#!/usr/bin/env python

import rospy
import numpy as np
##TODO: Include the header file for the three services defined in lab03task02srv

##TODO: Complete these functions
def convert_quat2zyx():
    print 'test'


def convert_quat2angleaxis():
    print 'test'


def convert_rotmat2quat():
    print 'test'


def rotation_converter():
    rospy.init_node('rotation_converter')
    ##TODO: Initialise the services
    rospy.spin()


if __name__ == "__main__":
    rotation_converter()
