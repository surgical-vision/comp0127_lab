#!/usr/bin/env python

import rospy
import rosbag
import rospkg
import sys
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Point
from cw2q4.youbotKine import youbot_kinematic


##TODO: You can design your code to achieve the q6 task however you want as long as it stays in this file and it runs.

def main_traj(cw2data):
    rospy.init_node('youbot_traj_cw2', anonymous=True)

    my_youbot = youbot_kinematic()

    rospack = rospkg.RosPack()
    path = rospack.get_path('cw2q6')

    bag = rosbag.Bag(path + '/bags/data' + str(cw2data) + '.bag')


    dt = 0.01
    joint_traj = JointTrajectory()
    ##TODO: Change this as appropriate
    if (int(cw2data) == 1):
        print("run q6a")
    else:
        print("run q6b")
    


if __name__ == '__main__':
    try:
        main_traj(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
