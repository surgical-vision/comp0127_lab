#!/usr/bin/env python

import rospy
import random
from sensor_msgs.msg import JointState
from kdl_kine.kdl_kine_solver import  robot_kinematic
from kdl_kine.urdf import *
import numpy as np

def show_jacobian():

    rospy.init_node('kuka_dynamics_node')

    rate = rospy.Rate(10)
    # Initiate the base link(where the kinematic chain starts) and the end-effector link(where it ends).
    # Please look at the urdf file of the respective robot arm for the names.
    iiwa_kine = robot_kinematic('iiwa_link_0', 'iiwa_link_ee')

    q = PyKDL.JntArray(7)
    qdot = PyKDL.JntArray(7)

    while not rospy.is_shutdown():

        
        #Because joint_state_publisher does not publish velocity, we manually generate random joint positions and velocities as input to B, C, g matrices.
        for i in range(7):
            q[i] = random.uniform(-1, 1)
            qdot[i] = random.uniform(-0.5, 0.5)

        print('The current joint position: ')
        print(q)
        print('The current joint velocity: ')
        print(qdot)
        print('The current robot pose: ')
        print(iiwa_kine.forward_kinematics(q))
        print('B(q): ')
        #PyKDL has a unique representation for the matrix B (only in Python, doesn't apply to C++), so we have to convert the representation to a matrix manually.
        B = iiwa_kine.getB(q)

        Bmat = np.zeros((7, 7))
        for i in range(7):
            for j in range(7):
                Bmat[i, j] = B[i, j]
        print(Bmat)

        print('C(q, qdot): ')
        print(iiwa_kine.getC(q, qdot))

        print('g(q): ')
        print(iiwa_kine.getG(q))

        rate.sleep()



if __name__ == '__main__':
    show_jacobian()
