#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from kdl_kine.kdl_kine_solver import  robot_kinematic
from lab06example01.hArmKine import hArm_kinematic



def show_jacobian():

    rospy.init_node('robotis_inverse_kine_node')

    rate = rospy.Rate(10)
    # Initiate the base link(where the kinematic chain starts) and the end-effector link(where it ends).
    # Please look at the urdf file of the respective robot arm for the names.
    h_kine = robot_kinematic('link1', 'end_link')
    h_kine_lab = hArm_kinematic()
    

    while not rospy.is_shutdown():

        current_frame_kdl = h_kine.forward_kinematics(h_kine.current_joint_position)
        current_frame_lab = h_kine_lab.forward_kine(h_kine_lab.current_joint_position, 6)

        print('The current robot pose (kdl): ')
        print(current_frame_kdl)
        print('Inverse kinematic solution (kdl): ')
        print(h_kine.inverse_kinematics_closed(current_frame_kdl))

        print('The current robot pose (lab): ')
        print(current_frame_lab)
        print('Inverse kinematic solution (lab): ')
        print(h_kine_lab.inverse_kine_closed_form(current_frame_lab))

        rate.sleep()



if __name__ == '__main__':
    show_jacobian()
