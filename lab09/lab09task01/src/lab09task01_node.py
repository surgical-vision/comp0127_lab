#!/usr/bin/env python3
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


class Iiwa14KDLDynamic:
    def __init__(self, tf_suffix='kdl'):
        self.tf_suffix = tf_suffix

        # Load kinematic chain from robot description
        (ok, kine_tree) = treeFromParam("robot_description")
        self.kine_chain = kine_tree.getChain("iiwa_link_0", "iiwa_link_ee")

        self.NJoints = self.kine_chain.getNrOfJoints()
        # KDL solvers
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kine_chain)
        self.dyn_solver = PyKDL.ChainDynParam(self.kine_chain, PyKDL.Vector(0, 0, -9.8))

        # Set current joint position
        self.previous_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.previous_t = 0

        # Set joint limits
        self.joint_limit_min = [-170 * pi / 180, -120 * pi / 180, -170 * pi / 180, -120 * pi / 180, -170 * pi / 180,
                                -120 * pi / 180, -175 * pi / 180]
        self.joint_limit_max = [170 * pi / 180, 120 * pi / 180, 170 * pi / 180, 120 * pi / 180, 170 * pi / 180,
                                120 * pi / 180, 175 * pi / 180]
        # TODO:
        # Setup transform broadcaster and joint states subscriber
        # Define a subscriber that will subscribe joint_states topic and will call print_BCG as a callback.
        # -- Your code starts here --

        # --  Your code ends here  --

    def compute_dynamics(self, msg):
        """ ROS callback function for joint states of the robot. Broadcasts the current pose of end effector and
        computes dynamics components with KDL.
        Args:
            msg (JointState): Joint state message containing current robot joint position.
        """
        # TODO:
        # Grab the positions from the JointState message and set to current_joint_position
        # -- Your code starts here --

        # --  Your code ends here  --

        # TODO:
        # Compute the current pose using the given forward kinematics function and broadcast the pose with the
        # self.broadcast_pose function
        # -- Your code starts here --

        # -- Your code ends here --
        self.broadcast_pose(current_pose)

        # TODO:
        # Compute the joint velocities based on the joint positions and the msg timestamp. Store the previous timestamp
        # and previous joint position
        # -- Your code starts here --

        # -- Your code ends here --
        self.previous_joint_position = current_joint_position
        self.previous_t = current_t

        print('The current joint position: ')
        print(current_joint_position)
        print('The current joint velocity: ')
        print(joint_velocities)
        print('The current robot pose: ')
        print(current_pose)
        print('B(q): ')
        print(self.get_B(current_joint_position))

        print('C(q, qdot) * qdot: ')
        print(self.get_C_times_qdot(current_joint_position, joint_velocities))

        print('g(q): ')
        print(self.get_G(current_joint_position))

    def forward_kinematics(self, joints_readings):
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

    def broadcast_pose(self, pose):
        """Given a pose transformation matrix, broadcast the pose to the TF tree.
        Args:
            pose (np.ndarray): Transformation matrix of pose to broadcast.

        """
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'iiwa_link_0'
        transform.child_frame_id = 'iiwa_ee_' + self.tf_suffix

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    def get_B(self, joint_readings):
        """Given the joint positions of the robot, compute inertia matrix B.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            B (numpy.ndarray): The output is a numpy 7*7 matrix describing the inertia matrix B.
        """
        q = PyKDL.JntArray(self.NJoints)
        KDL_B = PyKDL.JntSpaceInertiaMatrix(self.NJoints)

        # TODO:
        # Use the dynamic solver to compute the B matrix
        # For each q index of JntArray, set the that index of joint_readings. Use the kdl_jnt_array_to_list function.
        # Call self.dyn_solver with q and KDL_B
        # -- Your code starts here --

        # -- Your code ends here --
        return self.kdl_to_mat(KDL_B)

    def get_C_times_qdot(self, joint_readings, joint_velocities):
        """Given the joint positions and velocities of the robot, compute Coriolis terms C.
        Args:
            joint_readings (list): The positions of the robot joints.
            joint_velocities (list): The velocities of the robot joints.

        Returns:
            C (numpy.ndarray): The output is a numpy 7*1 matrix describing the Coriolis terms C times joint velocities.
        """
        q = PyKDL.JntArray(self.NJoints)
        qdot = PyKDL.JntArray(self.NJoints)
        KDL_C = PyKDL.JntArray(self.NJoints)

        # TODO:
        # Use the dynamic solver to compute the C matrix.
        # For each q and qdot index of JntArray, set the that index of joint_readings and joint_velocities.
        # Use the kdl_jnt_array_to_list function.
        # Call self.dyn_solver.JntToCoriolis with q, qdot and KDL_C
        # -- Your code starts here --

        # -- Your code ends here --
        return self.kdl_jnt_array_to_list(KDL_C)

    def get_G(self, joint_readings):
        """Given the joint positions of the robot, compute the gravity matrix g.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            G (numpy.ndarray): The output is a numpy 7*1 numpy array describing the gravity matrix g.
        """
        q = PyKDL.JntArray(self.NJoints)
        KDL_G = PyKDL.JntArray(self.NJoints)

        # TODO:
        # Use the dynamic solver to compute the C matrix.
        # For each q index of JntArray, set the that index of joint_readings. Use the kdl_jnt_array_to_list function.
        # Call self.dyn_solver.JntToCoriolis with q and KDL_G
        # -- Your code starts here --

        # -- Your code ends here --
        return self.kdl_jnt_array_to_list(KDL_G)

    @staticmethod
    def rotmat2q(T):
        """Convert rotation matrix to Quaternion.
        Args:
            T (np.ndarray): Rotation matrix to convert to Quaternion representation.

        Returns:
            q (Quaternion): Quaternion conversion of given rotation matrix.
        """
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

    @staticmethod
    def convert_kdl_frame_to_mat(frame):
        mat = np.identity(4)
        mat[:3, -1] = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        mat[:3, :3] = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
                                [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
                                [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]])
        return mat

    @staticmethod
    def kdl_to_mat(m):
        mat = np.mat(np.zeros((m.rows(), m.columns())))
        for i in range(m.rows()):
            for j in range(m.columns()):
                mat[i, j] = m[i, j]
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
        rospy.init_node('lab09task01', anonymous=True)

        iiwa14 = Iiwa14KDLDynamic()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
