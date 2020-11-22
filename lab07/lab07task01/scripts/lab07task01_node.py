#!/usr/bin/env python

import rospy
import rospkg
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lab06task02.youbotKine import youbot_kinematic

def youbot_traj():

    rospy.init_node('lab7youbot_traj')

    rospack = rospkg.RosPack()

    path = rospack.get_path('lab07example01')

    ##TODO: Initialise the bag and read the topic "my_transform" from the data
    ##TODO: Do the inverse kinematic to those transform to get the corresponding joints.
    ##TODO: Publish the JointTrajetory message to the corresponding topic to make the robot moves.


    rospy.sleep(5)


if __name__ == '__main__':
    try:
        youbot_traj()
    except rospy.ROSInterruptException:
        pass
