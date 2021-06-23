#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from kdl_kine.kdl_kine_solver import  robot_kinematic



def show_jacobian():

    rospy.init_node('robotis_jacob_node')

    rate = rospy.Rate(10)
    # Initiate the base link(where the kinematic chain starts) and the end-effector link(where it ends).
    # Please look at the urdf file of the respective robot arm for the names.
    h_kine = robot_kinematic('link1', 'end_link')


    while not rospy.is_shutdown():
        print('The current robot pose: ')
        print(h_kine.forward_kinematics(h_kine.current_joint_position))
        print('Jacobian: ')
        print(h_kine.get_jacobian(h_kine.current_joint_position))
        rate.sleep()



if __name__ == '__main__':
    show_jacobian()
