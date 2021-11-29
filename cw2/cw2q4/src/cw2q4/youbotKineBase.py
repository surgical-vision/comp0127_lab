import rospy
from math import pi
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory


class YoubotKinematicBase(object):
    def __init__(self, tf_suffix=''):
        # Robot variables
        # Identify class used when broadcasting tf with a suffix
        self.tf_suffix = tf_suffix
	
	# --> This was update on 20/11/2021. Feel free to use your own code.
        youbot_dh_parameters = {'a': [-0.033, 0.155, 0.135, +0.002, 0.0],
                                'alpha': [pi / 2, 0.0, 0.0, pi / 2, pi],
                                'd': [0.145, 0.0, 0.0, 0.0, -0.185],
                                'theta': [pi, pi / 2, 0.0, -pi / 2, pi]}

        self.dh_params = youbot_dh_parameters.copy()

        # Set current joint position
        self.current_joint_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # Set joint limits
        self.joint_limit_min = np.array([-169 * np.pi / 180, -65 * np.pi / 180, -150 * np.pi / 180,
                                         -102.5 * np.pi / 180, -167.5 * np.pi / 180])
        self.joint_limit_max = np.array([169 * np.pi / 180, 90 * np.pi / 180, 146 * np.pi / 180,
                                         102.5 * np.pi / 180, 167.5 * np.pi / 180])

        # ROS related
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                queue_size=5)
        self.traj_publisher = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                              queue_size=5)
        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

    def joint_state_callback(self, msg):
        """ ROS callback function for joint states of the robot. Broadcasts the current pose of end effector.

        Args:
            msg (JointState): Joint state message containing current robot joint position.

        """

        self.current_joint_position = list(msg.position)
        current_pose = self.forward_kinematics(self.current_joint_position)
        self.broadcast_pose(current_pose)

    def broadcast_pose(self, pose):
        """Given a pose transformation matrix, broadcast the pose to the TF tree.

        Args:
            pose (np.ndarray): Transformation matrix of pose to broadcast.

        """
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'arm_end_effector_' + self.tf_suffix

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    def forward_kinematics(self, joint_readings, up_to_joint=5):
        """This function solves forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters found in the
        init method and joint_readings.
        Args:
            joint_readings (list): the state of the robot joints. In a youbot those are revolute
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks.
                Defaults to 5.
        """
        raise NotImplementedError

    def get_jacobian(self, joint):
        """Compute Jacobian given the robot joint values. Implementation found in child classes.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute.
        Returns: Jacaobian matrix.

        """
        raise NotImplementedError

    @staticmethod
    def standard_dh(a, alpha, d, theta):
        """This function computes the homogeneous 4x4 transformation matrix T_i based on the four standard DH parameters
         associated with link i and joint i.
        Args:
            a ([int, float]): Link Length. The distance along x_i ( the common normal) between z_{i-1} and z_i
            alpha ([int, float]): Link twist. The angle between z_{i-1} and z_i around x_i.
            d ([int, float]): Link Offset. The distance along z_{i-1} between x_{i-1} and x_i.
            theta ([int, float]): Joint angle. The angle between x_{i-1} and x_i around z_{i-1}
        Returns:
            [np.ndarray]: the 4x4 transformation matrix T_i describing  a coordinate transformation from
            the concurrent coordinate system i to the previous coordinate system i-1
        """
        assert isinstance(a, (int, float)), "wrong input type for a"
        assert isinstance(alpha, (int, float)), "wrong input type for =alpha"
        assert isinstance(d, (int, float)), "wrong input type for d"
        assert isinstance(theta, (int, float)), "wrong input type for theta"
        A = np.zeros((4, 4))

	# --> This was updated on 20/11/2021. Feel free to use your own code.
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

        assert isinstance(A, np.ndarray), "Output wasn't of type ndarray"
        assert A.shape == (4, 4), "Output had wrong dimensions"
        return A

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
