#!/usr/bin/env python

import rospy
import rospkg
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def youbot_traj():

    rospy.init_node('lab7youbot_traj')

    rospack = rospkg.RosPack()

    path = rospack.get_path('lab07example01')
    bag = rosbag.Bag(path + '/../lab7data.bag')

    traj_pub = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=5)

    my_traj = JointTrajectory()


    tfs = 4
    my_traj.header.stamp = rospy.Time.now()
    my_traj.joint_names.append('arm_joint_1')
    my_traj.joint_names.append('arm_joint_2')
    my_traj.joint_names.append('arm_joint_3')
    my_traj.joint_names.append('arm_joint_4')
    my_traj.joint_names.append('arm_joint_5')

    for topic, msg, t in bag.read_messages(topics=['my_joint_states']): #Go through every message in the loaded bag.
        my_pt = JointTrajectoryPoint()

        if len(msg.position) != 0:

            for i in range(0, 5):
                my_pt.positions.append(msg.position[i])
                my_pt.velocities.append(msg.velocity[i]) #Try commenting-uncommenting these two lines to see the difference in the trajectory.
                my_pt.accelerations.append(0) #Try commenting-uncommenting these two lines to see the difference in the trajectory.

        my_pt.time_from_start.secs = tfs
        tfs = tfs + 4
        my_traj.points.append(my_pt) #Add position and velocity into the trajectory message.

    bag.close

    rospy.sleep(5)

    traj_pub.publish(my_traj)


if __name__ == '__main__':
    try:
        youbot_traj()
    except rospy.ROSInterruptException:
        pass
