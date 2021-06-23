#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory


class youbot_kinematic(object):

    def __init__(self):

        self.dh_params = [[-0.033, pi/2, 0.145, pi],
                          [0.155, 0.0, 0.0, pi/2],
                          [0.135, 0.0, 0.0, 0.0],
                          [-0.002, pi/2, 0.0, -pi/2],
                          [0.0, pi, -0.185, pi]]

        self.joint_offset = [170*pi/180, 65*pi/180, -146*pi/180, 102.5*pi/180, 167.5*pi/180]

        self.current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_limit_min = [-169*pi/180, -65*pi/180, -150*pi/180, -102.5*pi/180, -167.5*pi/180]
        self.joint_limit_max = [169*pi/180, 90*pi/180, 146*pi/180, 102.5*pi/180, 167.5*pi/180]

        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                        queue_size=5)
        self.traj_publisher = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                              queue_size=5)

        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

    def dh_matrix_standard(self, a, alpha, d, theta):
        A = np.zeros((4, 4))

        A[0, 0] = np.cos(theta)
        A[0, 1] = -np.sin(theta) * np.cos(alpha)
        A[0, 2] = np.sin(theta) * np.sin(alpha)
        A[0, 3] = a * np.cos(theta)

        A[1, 0] = np.sin(theta)
        A[1, 1] = np.cos(theta) * np.cos(alpha)
        A[1, 2] = -np.cos(theta) * np.sin(alpha)
        A[1, 3] = a * np.sin(theta)

        A[2, 1] = np.sin(alpha)
        A[2, 2] = np.cos(alpha)
        A[2, 3] = d

        A[3, 3] = 1.0

        return A

    def joint_state_callback(self, msg):
        for i in range(0, 5):
            self.current_joint_position[i] = msg.position[i]

        current_pose = self.forward_kine_offset(self.current_joint_position, 5)
        self.broadcast_pose(current_pose)


    def forward_kine(self, joint, frame):
        T = np.identity(4)

        for i in range(0, 5):
            A = self.dh_matrix_standard(self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2],
                                            joint[i] + self.dh_params[i][3])

            T = T.dot(A)

        return T


    def forward_kine_offset(self, joint, frame):
        T = np.identity(4)

        for i in range(0, 5):
            if (i == 0):
                A = self.dh_matrix_standard(self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2],
                                       self.joint_offset[i] - (joint[i] + self.dh_params[i][3]))
            else:
                A = self.dh_matrix_standard(self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2],
                                       (joint[i] + self.dh_params[i][3]) - self.joint_offset[i])

            T = T.dot(A)

        return T


    def broadcast_pose(self, pose):

        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'arm_end_effector'

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    def rotmat2q(self, T):
        q = Quaternion()

        angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1) / 2)

        xr = T[2, 1] - T[1, 2]
        yr = T[0, 2] - T[2, 0]
        zr = T[1, 0] - T[0, 1]

        x = xr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
        y = yr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
        z = zr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))

        q.w = np.cos(angle / 2)
        q.x = x * np.sin(angle / 2)
        q.y = y * np.sin(angle / 2)
        q.z = z * np.sin(angle / 2)

        return q

    def get_jacobian(self, joint):
        ##TODO: Fill in this function to complete the question 4a
        raise NotImplementedError() #Remove this line, once implemented everything

    def inverse_kine_closed_form(self, desired_pose):
        ##Ignore this.
        raise NotImplementedError() #Remove this line, once implemented everything

    def inverse_kine_ite(self, desired_pose, current_joint):
        ##TODO: Fill in this function to complete the question 4c
        raise NotImplementedError() #Remove this line, once implemented everything


    def check_singularity(self, current_joint):
        ##TODO: Fill in this function to complete the question 4d
        raise NotImplementedError() # Remove thie line, once implemented everything
