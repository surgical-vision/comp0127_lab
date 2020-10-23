#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros  import transform_broadcaster

##TODO: Complete the forward kinematic routine. The function should compute the transformation between each frame and use tf broadcaster to publish the transformation.
def forward_kinematic():
    print 'test'

def main():
    rospy.init_node('forward_kinematic_node')
    ##TODO: Initialise the subscriber to the topic "/joint_states" and its callback function forward_kinematic
    rospy.spin()


if __name__ == "__main__":
    main()