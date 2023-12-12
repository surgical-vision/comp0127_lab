#!/usr/bin/env python3
import numpy as np
import rospy
from cw2q4.youbotKineKDL import YoubotKinematicKDL


# Recall pose representation as x_e = [p_e, phi_e]^T, p_e = [p_ex, p_ey, p_ez]^T, phi_e = [r_ex, r_ey, r_ez]^T.
# Joints are represented as q = [q_1, q_2 ..., q_nN]^T, Lecture 5 slide 2
# For position only derivation says pdot_e = J_p qdot. Lecture 5 slide 18. [3x1] = [3xN][Nx1]
# Thus for the Jacobian transpose iterative approach would look something like:
# J = J[:3, :]
# q_k = q_(k-1) + alpha * J^T * (p_e_desired - x_e[:3])


def ik_position_only(kdl_youbot, pose, q0, alpha=1.0):
    """This function is an exmaple of how one would implement inverse kinematics for position only using the Jacobian
    transpose approach in a single iteration.
    Args:
        kdl_youbot (YoubotKDL object): YoubotKDL object that performs FK and jacobian calculations.
        pose (np.ndarray, 4x4): 4x4 transformations describing the pose of the end-effector position.
        q0 (np.ndarray, 5x1):A 5x1 array for the initial starting point of the algorithm.
        alpha (float): Step size.
        num (int): Number of interations before returning a solution.
    Returns:
        q (np.ndarray, 5x1): The IK solution for the given pose.
        error (float): The Cartesian error of the solution.
    """
    # Some useful notes:
    # We are only interested in the position control - take only position of the pose as well as elements of the
    # Jacobian that will affect the position of the error.

    # get desired position from the desired pose
    Pd = pose[:3, 3].ravel()
    q = q0
    J = kdl_youbot.get_jacobian(q0)[:3, :]
    # Take only first 3 rows as position only solution.
    P = np.array(kdl_youbot.forward_kinematics(q0))[:3, -1]
    e = Pd - P.ravel()
    q += alpha * np.matmul(J.T, e)
    error = np.linalg.norm(e)
    return q, error


if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node('lab08_example', anonymous=True)

        hardcoded_joint_target = [4.71, 1.38, -3.21, 1.79, 1.73]
        q_0 = [0, 0, 0, 0, 0]
        kdl_youbot = YoubotKinematicKDL()
        target_pose = kdl_youbot.forward_kinematics(hardcoded_joint_target)
        q, error = ik_position_only(kdl_youbot, target_pose, q_0)
        print("q: " + str(q))
        print("error: " + str(error))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass