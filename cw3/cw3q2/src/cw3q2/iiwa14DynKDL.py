#!/usr/bin/env python3

import numpy as np
import PyKDL
from cw3q2.iiwa14DynBase import Iiwa14DynamicBase
from kdl_parser_py.urdf import treeFromUrdfModel

from urdf_parser_py.urdf import URDF
import rospkg


class Iiwa14DynamicKDL(Iiwa14DynamicBase):
    def __init__(self, tf_suffix='kdl'):
        super(Iiwa14DynamicKDL, self).__init__(tf_suffix)

        # load from urdf file
        rospack = rospkg.RosPack()
        iiwa_description_path = rospack.get_path('cw3q2')
        robot = URDF.from_xml_file(iiwa_description_path + '/model.urdf')

        (ok, self.kine_tree) = treeFromUrdfModel(robot)

        self.kine_chain = self.kine_tree.getChain("iiwa_link_0", "iiwa_link_ee")
        self.NJoints = self.kine_chain.getNrOfJoints()
        # KDL solvers
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kine_chain)
        self.ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self.kine_chain)

        self.ik_solver = PyKDL.ChainIkSolverPos_NR_JL(self.kine_chain,
                                                      self.list_to_kdl_jnt_array(self.joint_limit_min),
                                                      self.list_to_kdl_jnt_array(self.joint_limit_max),
                                                      self.fk_solver, self.ik_v_kdl, 1000)
        self.jac_calc = PyKDL.ChainJntToJacSolver(self.kine_chain)
        self.dyn_solver = PyKDL.ChainDynParam(self.kine_chain, PyKDL.Vector(0, 0, -9.8))

    def forward_kinematics(self, joints_readings, up_to_joint=7):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters and
        joint_readings.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks.
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
    
    def get_B(self, joint_readings):
        """Given the joint positions of the robot, compute inertia matrix B.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            B (numpy.ndarray): The output is a numpy 7*7 matrix describing the inertia matrix B.
        """
        q = PyKDL.JntArray(self.NJoints)
        KDL_B = PyKDL.JntSpaceInertiaMatrix(self.NJoints)

        for i in range(0, self.kine_chain.getNrOfJoints()):
            q[i] = joint_readings[i]

        self.dyn_solver.JntToMass(q, KDL_B)
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

        for i in range(0, self.NJoints):
            q[i] = joint_readings[i]
            qdot[i] = joint_velocities[i]

        self.dyn_solver.JntToCoriolis(q, qdot, KDL_C)

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

        for i in range(0, self.NJoints):
            q[i] = joint_readings[i]

        self.dyn_solver.JntToGravity(q, KDL_G)
        return self.kdl_jnt_array_to_list(KDL_G)

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
