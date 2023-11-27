#!/usr/bin/env python
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cw2q4.youbotKineKDL import YoubotKinematicKDL
from visualization_msgs.msg import Marker

# Lab07Task01: Publishing a joint trajectory and CW2 Q6 Primer
# In this lab, the task is to complete the youbot_traj function. You are given hardcoded_joint_targets. The tasks are as
# follows.
# 1. Load the hardcoded_joint_targets into a numpy array
# 2. Compute the Cartesian checkpoints of the given joint_targets
# 3. Publish the checkpoints using the publish_checkpoints function
# 4. Publish a joint trajectory message with the joint targets. The youbot should then go to each checkpoint


def youbot_traj():
    rospy.init_node('lab07_youbot_traj')
    kdl_youbot = YoubotKinematicKDL()
    rospy.sleep(2.0)

    # Create trajectory publisher and a checkpoint publisher to visualize checkpoints
    traj_pub = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=5)
    checkpoint_pub = rospy.Publisher("checkpoint_positions", Marker, queue_size=5)

    # Joint values
    hardcoded_joint_targets = [[4.71, 1.38, -3.21, 1.79, 1.73], [1.44, 0.71, -2.51, 1.39, 1.6]]

    joint_targets = np.zeros((5, 2), dtype=float)

    # Given the hardcoded_joint_targets, load the joint targets into the numpy array variable joint_targets. Although
    # this is a somewhat unnecessary step, the template given in CW2 Q6 load_targets, loads the joint targets and
    # Cartesian checkpoints in this same way,
    # your code starts here ------------------------------
    for i in range(joint_targets.shape[1]):
        joint_targets[:, i] = hardcoded_joint_targets[i]
    # your code ends here ------------------------------

    # Compute the forward kinematics using KDL and publish the Cartesian positions of these checkpoints. The publish
    # checkpoint method needs the transformation matrix.
    # your code starts here ------------------------------
    checkpoints = np.zeros((4, 4, joint_targets.shape[1]), dtype=float)
    for i in range(joint_targets.shape[1]):
        checkpoints[:, :, i] = kdl_youbot.forward_kinematics(list(joint_targets[:, i]))
    # your code ends here ------------------------------

    rospy.sleep(2.0)

    # Call the publish_checkpoints function to publish the found Cartesian positions of the loaded joints
    # your code starts here ------------------------------
    publish_checkpoints(checkpoint_pub, checkpoints)
    # your code ends here ------------------------------

    # Create a trajectory message and publish to get the robot to move to this checkpoints
    traj = JointTrajectory()
    # your code starts here ------------------------------
    dt = 2
    t = 10
    for i in range(joint_targets.shape[1]):
        traj_point = JointTrajectoryPoint()
        traj_point.positions = joint_targets[:, i]
        t = t + dt
        traj_point.time_from_start.secs = t
        traj.points.append(traj_point)

    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
    traj_pub.publish(traj)
    # your code ends here ------------------------------


def publish_checkpoints(checkpoint_pub, tfs):
    """This function gets a np.ndarray of transforms and publishes them in a color coded fashion to show how the
    Cartesian path of the robot end-effector.
    Args:
        tfs (np.ndarray): A array of 4x4xn homogenous transformations specifying the end-effector trajectory.
    """
    id = 0
    for i in range(0, tfs.shape[2]):
        marker = Marker()
        marker.id = id
        id += 1
        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.b = 0.0
        marker.color.g = 0.0 + id * 0.05
        marker.color.r = 1.0 - id * 0.05
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = tfs[0, -1, i]
        marker.pose.position.y = tfs[1, -1, i]
        marker.pose.position.z = tfs[2, -1, i]
        checkpoint_pub.publish(marker)


if __name__ == '__main__':
    try:
        youbot_traj()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
