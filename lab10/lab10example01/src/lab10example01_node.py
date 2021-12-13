#!/usr/bin/env python
import numpy as np
import rospy
from math import pi
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion

import PyKDL
from kdl_parser_py.urdf import treeFromParam


# Lab09Task01: Calculate dynamic components B, C and g using KDL.

# In this lab example, using the iiwa14 robot, we will demonstrate how to use KDL to compute the various dynamic
# components. The script will grab the current joint configuration of the robot and compute the velocity. With these
# values, the script then performs forward kinematics with KDL as well as dynamics using KDL.


class Iiwa14ForwardKinematics:
    def __init__(self):
        # Load kinematic chain from robot description
        (ok, kine_tree) = treeFromParam("robot_description")
        self.kine_chain = kine_tree.getChain("iiwa_link_0", "iiwa_link_ee")

        self.NJoints = self.kine_chain.getNrOfJoints()
        # KDL solvers
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kine_chain)

        self.X_alpha = [pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, 0.0]
        self.Y_alpha = [pi, pi, 0, pi, 0, pi, 0]
        # The translation between each joint for manual forward kinematic (not using the DH convention). [x,y,z]
        self.translation_vec = np.array([[0, 0, 0.2025],
                                         [0, 0.2045, 0],
                                         [0, 0, 0.2155],
                                         [0, 0.1845, 0],
                                         [0, 0, 0.2155],
                                         [0, 0.081, 0],
                                         [0, 0, 0.045]])

        # The centre of mass of each link with respect to the preceding joint. [x,y,z]
        self.link_cm = np.array([[0, -0.03, 0.12],
                                 [0.0003, 0.059, 0.042],
                                 [0, 0.03, 0.13],
                                 [0, 0.067, 0.034],
                                 [0.0001, 0.021, 0.076],
                                 [0, 0.0006, 0.0004],
                                 [0, 0, 0.02]])

        # Define a subscriber that will subscribe joint_states topic and will compute forward kinematics and forward
        # kinematics at centre of mass.
        self.sub_fk = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback,
                                       queue_size=5)
        self.pose_broadcaster = TransformBroadcaster()

    def joint_states_callback(self, msg):
        """ ROS callback function for joint states of the robot. Compute the forward kinematics of the end-effector using
        non-DH and KDL for comparison. Also example of computing centre of mass forward kinematics.
        Args:
            msg (JointState): Joint state message containing current robot joint position.
        """
        # Grab the positions from the JointState message and set to current_joint_position
        current_joint_position = list(msg.position)

        # Compute the current pose using the given forward kinematics function and broadcast the pose with the
        # self.broadcast_pose function
        current_pose = self.forward_kinematics(current_joint_position)
        self.broadcast_pose(current_pose, 'ee_fk')
        current_pose_kdl = self.forward_kinematics_kdl(current_joint_position)
        self.broadcast_pose(current_pose_kdl, 'ee_kdl')

        # Compute the forward kinematics centre of mass
        for i in range(1, self.NJoints):
            com_fk = self.forward_kinematics_centre_of_mass(current_joint_position, up_to_joint=i)
            self.broadcast_pose(com_fk, 'com_' + str(i))

    def forward_kinematics(self, joints_readings, up_to_joint=7):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        joint. Reference Lecture 9 slide 13.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 7.
        Returns:
            np.ndarray The output is a numpy 4*4 matrix describing the transformation from the 'iiwa_link_0' frame to
            the selected joint frame.
        """

        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)

        T = np.identity(4)
        # iiwa base offset
        T[2, 3] = 0.1575

        # Some useful steps:
        # 1. You are given the location of each joint with translation_vec, X_alpha and Y_alpha. Also available are
        # function T_rotation_X, T_rotation_Y, T_rotation_Z, T_translation for rotation and translation matrices.
        # 2. Use a for loop to compute the final transformation.
        for i in range(0, up_to_joint):
            T = T.dot(self.T_rotationZ(joints_readings[i]))
            T = T.dot(self.T_translation(self.translation_vec[i, :]))
            T = T.dot(self.T_rotationX(self.X_alpha[i]))
            T = T.dot(self.T_rotationY(self.Y_alpha[i]))
        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"

        return T

    def forward_kinematics_centre_of_mass(self, joints_readings, up_to_joint=7):
        """This function computes the forward kinematics up to the centre of mass for the given joint frame.
        Reference - Lecture 9 slide 14.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 7.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint} for the
            centre of mass w.r.t the base of the robot.
        """
        T = np.identity(4)
        T[2, 3] = 0.1575

        T = self.forward_kinematics(joints_readings, up_to_joint - 1)
        T = T.dot(self.T_rotationZ(joints_readings[up_to_joint - 1]))
        T = T.dot(self.T_translation(self.link_cm[up_to_joint - 1, :]))
        return T

    def forward_kinematics_kdl(self, joints_readings):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters and
        joint_readings.
        Args:
            joints_readings (list): the state of the robot joints.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint}
                w.r.t the base of the robot.
        """
        # Convert joint readings to KDL JntArray
        joints_kdl = self.list_to_kdl_jnt_array(joints_readings)
        pose_kdl = PyKDL.Frame()
        self.fk_solver.JntToCart(joints_kdl, pose_kdl)
        # Convert KDL Pose to array
        pose = self.convert_kdl_frame_to_mat(pose_kdl)
        return pose

    def broadcast_pose(self, pose, suffix):
        """Given a pose transformation matrix, broadcast the pose to the TF tree.
        Args:
            pose (np.ndarray): Transformation matrix of pose to broadcast.

        """
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'iiwa_link_0'
        transform.child_frame_id = 'iiwa_' + suffix

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    @staticmethod
    def rotmat2q(T):
        """Convert rotation matrix to Quaternion.
        Args:
            T (np.ndarray): Rotation matrix to convert to Quaternion representation.

        Returns:
            q (Quaternion): Quaternion conversion of given rotation matrix.
        """
        q = Quaternion()
        m = T[:3, :3]
        tr = np.trace(m)
        if (tr > 0):
            s = np.sqrt(tr + 1.0) * 2.0
            q.w = 0.25 * s
            q.x = (m[2, 1] - m[1, 2]) / s
            q.y = (m[0, 2] - m[2, 0]) / s
            q.z = (m[1, 0] - m[0, 1]) / s
        elif ((m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2])):
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            q.w = (m[2, 1] - m[1, 2]) / s
            q.x = 0.25 * s
            q.y = (m[0, 1] + m[1, 0]) / s
            q.z = (m[0, 2] + m[2, 0]) / s
        elif (m[1, 1] > m[2, 2]):
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            q.w = (m[0, 2] - m[2, 0]) / s
            q.x = (m[0, 1] + m[1, 0]) / s
            q.y = 0.25 * s
            q.z = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            q.w = (m[1, 0] - m[0, 1]) / s
            q.x = (m[0, 2] + m[2, 0]) / s
            q.y = (m[1, 2] + m[2, 1]) / s
            q.z = 0.25 * s
        return q

    # Transformation matrices functions
    @staticmethod
    def T_translation(t):
        """ This function takes a translation vector t, [x,y,z] creates a transformation matrix T.

        Args:
            t: Translation vector to create transformation matrix.

        Returns:
            T np.ndarray: A 4x4 homogeneous transformation matrix.
        """
        T = np.identity(4)
        for i in range(0, 3):
            T[i, 3] = t[i]
        return T

    @staticmethod
    def T_rotationZ(theta):
        """ This function takes a rotation theta about z, creates a transformation matrix T.

        Args:
            theta: Rotation theta applied to z-axis.

        Returns:
            T np.ndarray: A 4x4 homogeneous transformation matrix.
        """
        T = np.identity(4)
        T[0, 0] = np.cos(theta)
        T[0, 1] = -np.sin(theta)
        T[1, 0] = np.sin(theta)
        T[1, 1] = np.cos(theta)
        return T

    @staticmethod
    def T_rotationX(theta):
        """ This function takes a rotation theta about x, creates a transformation matrix T.

        Args:
            theta: Rotation theta applied to x-axis.

        Returns:
            T np.ndarray: A 4x4 homogeneous transformation matrix.
        """
        T = np.identity(4)
        T[1, 1] = np.cos(theta)
        T[1, 2] = -np.sin(theta)
        T[2, 1] = np.sin(theta)
        T[2, 2] = np.cos(theta)
        return T

    @staticmethod
    def T_rotationY(theta):
        """ This function takes a rotation theta about y, creates a transformation matrix T.

        Args:
            theta: Rotation theta applied to y-axis.

        Returns:
            T np.ndarray: A 4x4 homogeneous transformation matrix.
        """
        T = np.identity(4)
        T[0, 0] = np.cos(theta)
        T[0, 2] = np.sin(theta)
        T[2, 0] = -np.sin(theta)
        T[2, 2] = np.cos(theta)
        return T

    # KDL conversion functions
    @staticmethod
    def convert_kdl_frame_to_mat(frame):
        mat = np.identity(4)
        mat[:3, -1] = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        mat[:3, :3] = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
                                [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
                                [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]])
        return mat

    @staticmethod
    def list_to_kdl_jnt_array(joints):
        kdl_array = PyKDL.JntArray(7)
        for i in range(0, 7):
            kdl_array[i] = joints[i]
        return kdl_array

    @staticmethod
    def kdl_jnt_array_to_list(kdl_array):
        joints = []
        for i in range(0, 7):
            joints.append(kdl_array[i])
        return joints


if __name__ == '__main__':
    # Initialize node
    try:
        rospy.init_node('lab10example01', anonymous=True)

        iiwa14 = Iiwa14ForwardKinematics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
