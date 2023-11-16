#!/usr/bin/env python3

import rospy
import numpy as np
from numpy import pi

# import the message type that holds data describing robot joint angles
from sensor_msgs.msg import JointState
# import the class that publishes coordinate frame transform information
from tf2_ros import TransformBroadcaster
# import the message type that expresses a transform from one coordinate frame to another
from geometry_msgs.msg import TransformStamped, Quaternion

import PyKDL as kdl
# import the kdl_parser_py library and get the treeFromParam function
# If not installed, can run sudo apt install ros-noetic-kdl-parser-py
from kdl_parser_py.urdf import treeFromParam


# The the YoubotKDLKinematic class provides the functionality to broadcast transforms using KDL.
# First, fill in the missing code for to perform forward kinematics with KDL and the robot description URDF.
# Second, test your DH parameters similarly as given in lab05_example.py
# These might not align perfectly - the figure given in the coursework is not a perfect representation.
class YoubotKDL:
    def __init__(self, kdl_kine_chain, tf_suffix):
        # Set class inputs
        self.kine_chain = kdl_kine_chain
        self.tf_suffix = tf_suffix
        # Setup transform broadcaster and joint states subscriber
        self.br = TransformBroadcaster()


        self.sub_fk = rospy.Subscriber('/joint_states', JointState, self.fkine_wrapper)

        # TODO: Create a subscriber that subscribes /joint_states and uses 'print_jacobian()' to compute and print the jacobian of our Youobot.
        #
        #

        # FK KDL solver
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kine_chain)

        # TODO: create a KDL jacobian solver that takes a kinematic chain and outputs the Jacobian
        # You may look here: http://docs.ros.org/en/electric/api/orocos_kdl/html/classKDL_1_1Jacobian.html (C++)
        # Or here: http://docs.ros.org/en/diamondback/api/kdl/html/python/kinematic_solvers.html (Python)
        #
        #
        #

    # This function takes in joint readings, converts to appropriate KDL types
    # then calls the FK solver to get the transformation
    def forward_kinematics(self, joints_readings):
        # Convert joint readings to KDL JntArray
        joints_kdl = self.list_to_kdl_jnt_array(joints_readings)
        pose_kdl = kdl.Frame()
        self.fk_solver.JntToCart(joints_kdl, pose_kdl)
        # Convert KDL Pose to array
        pose = self.convert_kdl_frame_to_mat(pose_kdl)
        return pose

    # TODO: Fill the inputs of this function.
    def print_jacobian(self, FILL_ME):
        j = self.get_jacobian('FILL ME')
        print(j)

    # TODO: Here you will have to use the jacobian solver object you have created in __init__ to compute the jacobian.
    # What are the inputs? Pay attention to the output data type. You'll have to use 'convert_kdl_to_mat' to convert it to a numpy array.
    def get_jacobian(self, FILL_ME):
        #
        #
        #
        return FILL_ME

    # This function is a callback for the subscriber for joint_states.
    # The transformation is computed with forward kinematics then the transform is broadcasted.
    def fkine_wrapper(self, joint_msg):
        transform = TransformStamped()

        T0ee = self.forward_kinematics(joint_msg.position)

        # Here we define the transform from joint frame 0 to joint frame 1 using the tf2 broadcaster.
        # define the transform timestamp
        transform.header.stamp = rospy.Time.now()

        ## Change the frame_id here appropriately depending on your kinematic chain. ##
        # define the transform header frame (parent frame)
        transform.header.frame_id = 'base_link'
        ## Change the frame_id here appropriately depending on your kinematic chain. ##

        # define the transform child frame
        transform.child_frame_id = self.tf_suffix + '_link'
        # populate the transform field. It consists of translation and rotation.
        transform.transform.translation.x = T0ee[0, 3]
        transform.transform.translation.y = T0ee[1, 3]
        transform.transform.translation.z = T0ee[2, 3]
        transform.transform.rotation = self.rotmat2q(T0ee)

        # broadcast the transform to tf2
        self.br.sendTransform(transform)

    @staticmethod
    def rotmat2q(T):
        # function that transforms a rotation matrix to quaternion representation
        q = Quaternion()
        angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1) / 2)

        if (angle == 0):
            q.w = 1
            q.x = 0
            q.y = 0
            q.z = 0
        else:
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
        # Helper function to convert kdl frames to numpy arrays
        mat = np.identity(4)
        mat[:3, -1] = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        mat[:3, :3] = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
                                [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
                                [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]])
        return mat

    @staticmethod
    def list_to_kdl_jnt_array(joints):
        # Helper function to convert joint list to kdl array
        kdl_array = kdl.JntArray(5)
        for i in range(0, 5):
            kdl_array[i] = joints[i]
        return kdl_array

    @staticmethod
    def convert_kdl_to_mat(data):
        mat = np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i, j] = data[i, j]
        return mat


# Create a KDL kinematic chain with the URDF files (parameter from robot description).
def kine_chain_urdf():

    ## Load in the URDF correctly from the robot description and create the kinematic chain##
    # Load DH directly from URDF file (see launch file for robot description)
    (ok, kine_tree) = treeFromParam("robot_description")
    kine_chain = kine_tree.getChain("base_link", "arm_link_ee")
    ## Load in the URDF correctly from the robot description and create the kinematic chain##

    return kine_chain


def main():
    rospy.init_node('youbot_KDL_forward_kinematic_node')

    ## Create the URDF_chain by calling the kine_chain_urdf function and then create the YoubotKDLKinematic class ##
    urdf_chain = kine_chain_urdf()
    urdf_ = YoubotKDL(urdf_chain, 'urdf')
    ## Create the URDF_chain by calling the kine_chain_urdf function and then create the YoubotKDLKinematic class ##

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
