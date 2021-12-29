import rospy
from math import pi
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory


class Iiwa14DynamicBase(object):
    def __init__(self, tf_suffix=''):
        # Robot variables
        # Identify class used when broadcasting tf with a suffix
        self.tf_suffix = tf_suffix
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

        # The mass of each link.
        self.mass = [4, 4, 3, 2.7, 1.7, 1.8, 0.3]

        # Moment on inertia of each link, defined at the centre of mass.
        # Each row is (Ixx, Iyy, Izz) and Ixy = Ixz = Iyz = 0.
        self.Ixyz = np.array([[0.1, 0.09, 0.02],
                              [0.05, 0.018, 0.044],
                              [0.08, 0.075, 0.01],
                              [0.03, 0.01, 0.029],
                              [0.02, 0.018, 0.005],
                              [0.005, 0.0036, 0.0047],
                              [0.001, 0.001, 0.001]])
        # gravity
        self.g = 9.8

        # Set current joint position
        self.current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Set joint limits
        self.joint_limit_min = [-170 * pi / 180, -120 * pi / 180, -170 * pi / 180, -120 * pi / 180, -170 * pi / 180,
                                -120 * pi / 180, -175 * pi / 180]
        self.joint_limit_max = [170 * pi / 180, 120 * pi / 180, 170 * pi / 180, 120 * pi / 180, 170 * pi / 180,
                                120 * pi / 180, 175 * pi / 180]

        # ROS related
        self.joint_state_sub = rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_state_callback,
                                                queue_size=5)
        self.traj_publisher = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                              queue_size=5)
        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

    def joint_state_callback(self, msg):
        """ ROS callback function for joint states of the robot. Broadcasts the current pose of end effector.

        Args:
            msg (JointState): Joint state message containing current robot joint position.

        """
        for i in range(0, 7):
            self.current_joint_position[i] = msg.position[i]
            self.current_joint_velocity[i] = msg.velocity[i]

        current_pose = self.forward_kinematics(self.current_joint_position, 7)
        self.broadcast_pose(current_pose)

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

    def forward_kinematics(self, joint_readings, up_to_joint=7):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        frame number. TODO: Explain new parameters for offsets
        Args:
            joint_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 5.
        """
        raise NotImplementedError

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

    def get_B(self, joint_readings):
        """Given the joint positions of the robot, compute inertia matrix B.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            B (numpy.ndarray): The output is a numpy 7*7 matrix describing the inertia matrix B.
        """
        raise NotImplementedError

    def get_C_times_qdot(self, joint_readings, joint_velocities):
        """Given the joint positions and velocities of the robot, compute Coriolis terms C.
        Args:
            joint_readings (list): The positions of the robot joints.
            joint_velocities (list): The velocities of the robot joints.

        Returns:
            C (numpy.ndarray): The output is a numpy 7*1 matrix describing the Coriolis terms C times joint velocities.
        """
        raise NotImplementedError

    def get_G(self, joint_readings):
        """Given the joint positions of the robot, compute the gravity matrix g.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            G (numpy.ndarray): The output is a numpy 7*1 numpy array describing the gravity matrix g.
        """
        raise NotImplementedError

    def rotmat2rodrigues(self, T):
        """Convert rotation matrix to rodrigues vector. Done by first converting to quaternion then to rodrigues.

        Args:
            T (np.ndarray): Rotation matrix to convert to rodrigues representation.

        Returns:
            p (np.ndarray): An array for the first 5 elements specifying translation and the last three specifying
        rotation.
        """
        assert isinstance(T, np.ndarray)

        p = np.empty(6, float)
        # First convert transformation matrix to quaternion
        q = self.rotmat2q(T)

        if q.w == 1 or q.w == -1:
            rx = 0
            ry = 0
            rz = 0
        else:
            theta = np.arccos(q.w / 2)
            ux = q.x / np.sqrt(1 - q.w ** 2)
            uy = q.y / np.sqrt(1 - q.w ** 2)
            uz = q.z / np.sqrt(1 - q.w ** 2)

            rx = ux * theta
            ry = uy * theta
            rz = uz * theta
        # Orientation of pose
        p[3] = rx
        p[4] = ry
        p[5] = rz
        # Translation of pose
        p[0] = T[0, 3]
        p[1] = T[1, 3]
        p[2] = T[2, 3]
        return p

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
