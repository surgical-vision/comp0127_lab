#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from kdl_kine.urdf import *

class robot_kinematic(object):
    def __init__(self, base_link, ee_link):

        # Setup the subscribers for the joint states
        self.subscriber_joint_state_ = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                        queue_size=5)

        # PyKDL
        self.kine_tree = PyKDL.Tree()
        self.current_pose = PyKDL.Frame()

        (ok, self.kine_tree) = treeFromParam("robot_description")

        self.kine_chain = self.kine_tree.getChain(base_link, ee_link)
        self.NJoints = self.kine_chain.getNrOfJoints()
        self.current_joint_position = PyKDL.JntArray(self.NJoints)
        self.current_joint_velocity = PyKDL.JntArray(self.NJoints)
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kine_chain)
        self.ik_solver = PyKDL.ChainIkSolverPos_LMA(self.kine_chain)
        self.jac_calc = PyKDL.ChainJntToJacSolver(self.kine_chain)
        self.dyn_solver = PyKDL.ChainDynParam(self.kine_chain, PyKDL.Vector(0, 0, -9.8))

    def joint_state_callback(self, msg):
        # Copies joint position into KDL JntArray
        for i in range(0, self.NJoints):
            self.current_joint_position[i] = msg.position[i]
            self.current_joint_velocity[i] = msg.velocity[i]

    def forward_kinematics(self, joint):
        #This "joint" is PyKDL.JntArray. The output is a PyKDL.Frame. 
        pose = PyKDL.Frame()
        self.fk_solver.JntToCart(joint, pose)
        return pose

    def get_jacobian(self, joint):
        #This "joint" is PyKDL.JntArray. The output is a PyKDL.Jacobian. 
        jac = PyKDL.Jacobian(self.NJoints)
        self.jac_calc.JntToJac(joint, jac)
        return jac

    def inverse_kinematics_closed(self, desired_pose):
        #This "desired_pose" is PyKDL.Frame. The output is a PyKDL.JntArray. 
        required_joint = PyKDL.JntArray(self.NJoints)
        # Current joint array, desired frame, required joint
        self.ik_solver.CartToJnt(self.current_joint_position, desired_pose, required_joint)
        return required_joint


    def getB(self, joint_val):
        #This "joint_val" is an array. The output is a numpy 2D array. 
        q = PyKDL.JntArray(self.NJoints)
        KDL_B = PyKDL.JntSpaceInertiaMatrix(self.NJoints)

        for i in range(0, self.kine_chain.getNrOfJoints()):
            q[i] = joint_val[i]

        self.dyn_solver.JntToMass(q, KDL_B)
        return KDL_B


    def getC(self, joint_val, joint_vel):
        #This "joint_val" and "joint_vel" are 1D arrays. The output is a JntArray.
        q = PyKDL.JntArray(self.NJoints)
        qdot = PyKDL.JntArray(self.NJoints)
        KDL_C = PyKDL.JntArray(self.NJoints)

        for i in range(0, self.NJoints):
            q[i] = joint_val[i]
            qdot[i] = joint_vel[i]

        self.dyn_solver.JntToCoriolis(q, qdot, KDL_C)

        return KDL_C


    def getG(self, joint_val):
        #This "joint_val" is a 1D array. The output is a JntArray.
        q = PyKDL.JntArray(self.NJoints)
        KDL_G = PyKDL.JntArray(self.NJoints)

        for i in range(0, self.NJoints):
            q[i] = joint_val[i]

        self.dyn_solver.JntToGravity(q, KDL_G)
        return KDL_G

