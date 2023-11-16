#!/usr/bin/env python3

import numpy as np
import PyKDL
from cw2q4.youbotKineBase import YoubotKinematicBase
from kdl_parser_py.urdf import treeFromUrdfModel

from urdf_parser_py.urdf import URDF
import rospkg


class YoubotKinematicKDL(YoubotKinematicBase):
    def __init__(self, tf_suffix='kdl'):
        super(YoubotKinematicKDL, self).__init__(tf_suffix)

        # load from urdf file
        rospack = rospkg.RosPack()
        youbot_description_path = rospack.get_path('cw2q4')
        robot = URDF.from_xml_file(youbot_description_path + '/model.urdf')

        (ok, self.kine_tree) = treeFromUrdfModel(robot)

        self.kine_chain = self.kine_tree.getChain("base_link", "arm_link_ee")
        self.NJoints = self.kine_chain.getNrOfJoints()
        self.current_joint_position = PyKDL.JntArray(self.NJoints)
        self.current_joint_velocity = PyKDL.JntArray(self.NJoints)
        # KDL solvers
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kine_chain)        
        self.jac_calc = PyKDL.ChainJntToJacSolver(self.kine_chain)

    def get_jacobian(self, joint):
        """Compute the jacobian matrix using KDL library.

        Args:
            joint (numpy.array): NumPy array of size 5 corresponding to number of joints on the robot.

        Returns:
            Jacobian (numpy.ndarray): NumPy matrix of size 6x5.

        """
        joints_kdl = self.list_to_kdl_jnt_array(joint)
        jac_kdl = PyKDL.Jacobian(self.kine_chain.getNrOfJoints())
        self.jac_calc.JntToJac(joints_kdl, jac_kdl)
        jac = self.convert_kdl_to_mat(jac_kdl)
        return jac

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters and
        joint_readings.
        Args:
            dh_dict (dict): A dictionary containing the dh parameters describing the robot
            joints_readings (list): the state of the robot joints. In a youbot those are revolute
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 5.
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

    @staticmethod
    def convert_kdl_frame_to_mat(frame):
        mat = np.identity(4)
        mat[:3, -1] = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        mat[:3, :3] = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
                                [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
                                [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]])
        return mat

    @staticmethod
    def convert_kdl_to_mat(data):
        mat = np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i, j] = data[i, j]
        return mat

    @staticmethod
    def list_to_kdl_jnt_array(joints):
        kdl_array = PyKDL.JntArray(5)
        for i in range(0, 5):
            kdl_array[i] = joints[i]
        return kdl_array

    @staticmethod
    def kdl_jnt_array_to_list(kdl_array):
        joints = []
        for i in range(0, 5):
            joints.append(kdl_array[i])
        return joints
