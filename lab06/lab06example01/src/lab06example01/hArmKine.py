#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped


class hArm_kinematic(object):

    def __init__(self):
        self.dh_params = [[0.0, -pi/2, 0.159, 0.0],
                          [0.26569, 0.0, 0.0, -pi/2+np.arctan(0.03/0.264)],
                          [0.03, -pi/2, 0.0, -np.arctan(0.03/0.264)],
                          [0.0, -pi/2, 0.258, 0.0],
                          [0.0, -pi/2, 0.0, 0.0],
                          [0.0, 0.0, -0.123, 0.0]]

        self.joint_limit_min = [-pi, -pi/2, -pi/2, -pi, -pi/2, -pi]
        self.joint_limit_max = [pi, pi/2, 3*pi/4, pi, pi/2, pi]
        self.current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                        queue_size=5)
        
        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

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


    def broadcast_pose(self, T):
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'world'
        transform.child_frame_id = self.joint_names[5]

        transform.transform.translation.x = T[0, 3]
        transform.transform.translation.y = T[1, 3]
        transform.transform.translation.z = T[2, 3]
        transform.transform.rotation = self.rotmat2q(T)

        self.pose_broadcaster.sendTransform(transform)

    def joint_state_callback(self, msg):
        
        for i in range(0, 6):
            self.current_joint_position[i] = msg.position[i]

        current_pose = self.forward_kine_offset(self.current_joint_position, 6)
        self.broadcast_pose(current_pose)


    def rotmat2q(self, T):
        q = Quaternion()

        angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1) / 2)

        xr = T[2, 1] - T[1, 2]
        yr = T[0, 2] - T[2, 0]
        zr = T[1, 0] - T[0, 1]

        if ((xr == 0) and (yr == 0) and (zr == 0)):

            q.w = 1.0
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0

        else:

            x = xr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            y = yr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            z = zr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))

            q.w = np.cos(angle / 2)
            q.x = x * np.sin(angle / 2)
            q.y = y * np.sin(angle / 2)
            q.z = z * np.sin(angle / 2)

        return q


    def forward_kine(self, joint, frame):
        #This function expects an offset-free joint value.
        T = np.identity(4)

        for i in range(0, frame):
            A = self.dh_matrix_standard(self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2], self.dh_params[i][3] + joint[i])

            T = T.dot(A)

        return T


    def forward_kine_offset(self, joint, frame):
        #This function expects a joint value with offset.
        T = np.identity(4)

        for i in range(0, frame):
            if (i == 4) or (i == 5):
                A = self.dh_matrix_standard(self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2], self.dh_params[i][3] - joint[i])
            else:
                A = self.dh_matrix_standard(self.dh_params[i][0], self.dh_params[i][1], self.dh_params[i][2], self.dh_params[i][3] + joint[i])

            T = T.dot(A)

        return T

    def inverse_kine_closed_form(self, pose):

        ##There are 8 possible inverse kinematic solutions. The last column indicates whether or not the solution lies within the robot workspace (does not exceed joint limit)
        inv_kine_sol = np.zeros((8, 7))

        a2 = self.dh_params[1][0]
        a3 = self.dh_params[2][0]
        d1 = self.dh_params[0][2]
        d4 = self.dh_params[3][2]
        d6 = self.dh_params[5][2]
        beta = np.arctan2(d4, a3)

        ##Rename the variables so that the code is more readable.
        x = pose[0, 3];
        y = pose[1, 3];
        z = pose[2, 3];
        r13 = pose[0, 2];
        r23 = pose[1, 2];
        r33 = pose[2, 2];

        ##Solve for theta1
        inv_kine_sol[0, 0] = inv_kine_sol[1, 0] = inv_kine_sol[2, 0] = inv_kine_sol[3, 0] = np.arctan2(y - d6*r23, x - d6*r13)

        ##Check with the joint limit for the other solution
        if (inv_kine_sol[0, 0] <= 0):
            inv_kine_sol[4, 0] = inv_kine_sol[5, 0] = inv_kine_sol[6, 0] = inv_kine_sol[7, 0] = np.arctan2(y - d6*r23, x - d6*r13) + pi
        else:
            inv_kine_sol[4, 0] = inv_kine_sol[5, 0] = inv_kine_sol[6, 0] = inv_kine_sol[7, 0] = np.arctan2(y - d6*r23, x - d6*r13) - pi


        k = K = 0

        ##Solve for theta3
        for i in range(0, 2):

            ##Rename the variables so that the code is more readable.
            th1 = inv_kine_sol[4*i, 0]

            if (np.sin(th1) == 0):
                k = (x - d6*r13)/np.cos(th1)
            else:
                k = (y - d6*r23)/np.sin(th1)

            K = k**2 + (z - d1 - d6*r33)**2 - a3**2 - a2**2 - d4**2
            K = K/(2*a2*np.sqrt(a3**2 + d4**2))

            inv_kine_sol[4*i, 2] = inv_kine_sol[4*i + 1, 2] = np.arccos(K) - beta + np.arctan(0.03/0.264)
            inv_kine_sol[4*i + 2, 2] = inv_kine_sol[4*i + 3, 2] = -np.arccos(K) - beta + np.arctan(0.03/0.264)

            ##Rename the variables so that the code is more readable.
            th3_1_with_offset = inv_kine_sol[4*i, 2] - np.arctan(0.03/0.264)
            th3_2_with_offset = inv_kine_sol[4*i + 2, 2] - np.arctan(0.03/0.264)

            B1 = a2 + a3*np.cos(th3_1_with_offset) - d4*np.sin(th3_1_with_offset)
            B2 = a2 + a3*np.cos(th3_2_with_offset) - d4*np.sin(th3_2_with_offset)

            A1 = a3*np.sin(th3_1_with_offset) + d4*np.cos(th3_1_with_offset)
            A2 = a3*np.sin(th3_2_with_offset) + d4*np.cos(th3_2_with_offset)

            ##Rename the variables so that the code is more readable.
            gamma_1 = np.arctan2(A1, B1)
            gamma_2 = np.arctan2(A2, B2)

            ##Solve for theta2
            inv_kine_sol[4*i, 1] = inv_kine_sol[4*i + 1, 1] = np.arctan2(k, z - d1 - d6*r33) - gamma_1 - np.arctan(0.03/0.264)
            inv_kine_sol[4*i + 2, 1] = inv_kine_sol[4*i + 3, 1] = np.arctan2(k, z - d1 - d6*r33) - gamma_2 - np.arctan(0.03/0.264)


        for i in range(0, 7):

            pose36 = np.linalg.inv(self.forward_kine(inv_kine_sol[i, :], 3)).dot(pose)


            ##Rename the variables so that the code is more readable.
            n13 = pose36[0, 2]
            n23 = pose36[1, 2]
            n31 = pose36[2, 0]
            n32 = pose36[2, 1]
            n33 = pose36[2, 2]

            ##Solve for theta5
            if (i%2 == 0):
                inv_kine_sol[i, 4] = np.arccos(-n33)
            else:
                inv_kine_sol[i, 4] = -np.arccos(-n33)

            ##Rename the variables so the code is more readable.
            th5 = inv_kine_sol[i, 4]

            ##Solve for theta4
            inv_kine_sol[i, 3] = np.arctan2(-n23/np.sin(th5), -n13/np.sin(th5))

            ##Solve for theta6
            inv_kine_sol[i, 5] = np.arctan2(n32/np.sin(th5), -n31/np.sin(th5))

            inv_kine_sol[i, 5] = -inv_kine_sol[i, 5]
            inv_kine_sol[i, 4] = -inv_kine_sol[i, 4]

        ##Check if the inverse kinematic solutions are in the robot workspace
        for i in range(0, 8):
            for j in range(0, 6):
                if (inv_kine_sol[i, j] <= self.joint_limit_min[j]) or (inv_kine_sol[i, j] >= self.joint_limit_max[j]):

                    ##If the solution is very close to the limit, it could be because of numerical error.
                    if (abs(inv_kine_sol[i, j] - self.joint_limit_min[j]) < 1.5*pi/180):
                        inv_kine_sol[i, j] = self.joint_limit_min[j]
                    elif (abs(inv_kine_sol[i, j] - self.joint_limit_max[j]) < 1.5*pi/180):
                        inv_kine_sol[i, j] = self.joint_limit_max[j]
                    else:
                        ##Put some value there to let you know that this solution is not valid.
                        inv_kine_sol[i, 6] = -j
                        break
                else:
                    ##Put some value there to let you know that this solution is valid.
                    inv_kine_sol[i, 6] = 1


        return inv_kine_sol

    def get_jacobian(self, joint):
        ##SINCE THE IMPLEMENTATION OF THIS FUNCTION IS VERY SIMILAR TO THE EXERCISES IN THE LAB, THIS IS LEFT BLANK ON PURPOSE.
        raise NotImplementedError()

    def inverse_kine_ite(self, pose, joint):
        ##SINCE THE IMPLEMENTATION OF THIS FUNCTION IS VERY SIMILAR TO THE EXERCISES IN THE LAB, THIS IS LEFT BLANK ON PURPOSE.
        raise NotImplementedError()

    def check_singularity(self, joint):
        ##SINCE THE IMPLEMENTATION OF THIS FUNCTION IS VERY SIMILAR TO THE EXERCISES IN THE LAB, THIS IS LEFT BLANK ON PURPOSE.
        raise NotImplementedError()
