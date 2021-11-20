#!/usr/bin/env python

import numpy as np
from youbotKineBase import YoubotKinematicBase


class YoubotKinematicStudent(YoubotKinematicBase):
    def __init__(self):
        super(YoubotKinematicStudent, self).__init__(tf_suffix='student')

        # Set the offset for theta
        youbot_joint_offsets = [170.0 * np.pi / 180.0,
                                -65.0 * np.pi / 180.0,
                                146 * np.pi / 180,
                                -102.5 * np.pi / 180,
                                -167.5 * np.pi / 180]

        # Apply joint offsets to dh parameters
        self.dh_params['theta'] = [theta + offset for theta, offset in
                                   zip(self.dh_params['theta'], youbot_joint_offsets)]

        # Joint reading polarity signs
        self.youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters and
        joint_readings.
        Args:
            joints_readings (list): the state of the robot joints. In a youbot those are revolute
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous tranformation matrix describing the pose of frame_{up_to_joint}
                w.r.t the base of the robot.
        """
        assert isinstance(self.dh_params, dict)
        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)
        assert up_to_joint >= 0
        assert up_to_joint <= len(self.dh_params['a'])

        T = np.identity(4)
        # Apply offset and polarity to joint readings (found in URDF file)
        joints_readings = [sign * angle for sign, angle in zip(self.youbot_joint_readings_polarity, joints_readings)]
        # your code starts here ------------------------------

        for i in range(up_to_joint):
            A = self.standard_dh(self.dh_params['a'][i],
                                 self.dh_params['alpha'][i],
                                 self.dh_params['d'][i],
                                 self.dh_params['theta'][i] + joints_readings[i])
            T = T.dot(A)

        # your code ends here -------------------------------
        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"
        return T

    def get_jacobian(self, joint):
        """Given the joint values of the robot, compute the Jacobian matrix. Coursework 2 Question 4a.
        Reference - Lecture 5 slide 24.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            Jacobian (numpy.ndarray): NumPy matrix of size 6x5 which is the Jacobian matrix.
        """
        assert isinstance(joint, list)
        assert len(joint) == 5

        # Your code starts here ----------------------------
        T1 = self.forward_kinematics(joint, 1)
        T2 = self.forward_kinematics(joint, 2)
        T3 = self.forward_kinematics(joint, 3)
        T4 = self.forward_kinematics(joint, 4)
        T5 = self.forward_kinematics(joint, 5)

        # Bit of trick here to match Jacobian of KDL with z0 being [0, 0, -1] since that is how its define in the URDF
        # instead of [0, 0, 1]
        z0 = np.array([0, 0, -1])
        z1 = np.array([T1[0, 2], T1[1, 2], T1[2, 2]])
        z2 = np.array([T2[0, 2], T2[1, 2], T2[2, 2]])
        z3 = np.array([T3[0, 2], T3[1, 2], T3[2, 2]])
        z4 = np.array([T4[0, 2], T4[1, 2], T4[2, 2]])

        o0 = np.array([0, 0, 0])
        o1 = np.array([T1[0, 3], T1[1, 3], T1[2, 3]])
        o2 = np.array([T2[0, 3], T2[1, 3], T2[2, 3]])
        o3 = np.array([T3[0, 3], T3[1, 3], T3[2, 3]])
        o4 = np.array([T4[0, 3], T4[1, 3], T4[2, 3]])
        Pe = np.array([T5[0, 3], T5[1, 3], T5[2, 3]])

        JP1 = np.cross(z0, np.subtract(Pe, o0))
        JP2 = np.cross(z1, np.subtract(Pe, o1))
        JP3 = np.cross(z2, np.subtract(Pe, o2))
        JP4 = np.cross(z3, np.subtract(Pe, o3))
        JP5 = np.cross(z4, np.subtract(Pe, o4))

        J1 = np.hstack((JP1, z0))
        J2 = np.hstack((JP2, z1))
        J3 = np.hstack((JP3, z2))
        J4 = np.hstack((JP4, z3))
        J5 = np.hstack((JP5, z4))
        jacobian = np.transpose(np.vstack((J1, J2, J3, J4, J5)))
        # Your code ends here ------------------------------
        assert jacobian.shape == (6, 5)
        return jacobian

    def check_singularity(self, joint):
        """Check for singularity condition given robot joints. Coursework 2 Question 4d.
        Reference Lecture 5 slide 30.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            singularity (bool): True if in singularity and False if not in singularity.

        """
        assert isinstance(joint, list)
        assert len(joint) == 5
        # Your code starts here ----------------------------

        Jacobian = self.get_jacobian(joint)
        product = np.dot(np.transpose(Jacobian), Jacobian)
        if (np.linalg.det(product) < 0.00000001):
            singularity = True
        else:
            singularity = False

        # Your code ends here ------------------------------
        assert isinstance(singularity, bool)
        return singularity
