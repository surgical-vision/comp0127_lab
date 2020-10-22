#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64


def joint_pub():
    rospy.init_node('trajectory_generator', anonymous=True)

    pub1 = rospy.Publisher('/EffortJointInterface_J1_controller/command', Float64, queue_size=100)
    pub2 = rospy.Publisher('/EffortJointInterface_J2_controller/command', Float64, queue_size=100)
    pub3 = rospy.Publisher('/EffortJointInterface_J3_controller/command', Float64, queue_size=100)
    pub4 = rospy.Publisher('/EffortJointInterface_J4_controller/command', Float64, queue_size=100)

    msg1 = Float64()
    msg2 = Float64()
    msg3 = Float64()
    msg4 = Float64()

    while not rospy.is_shutdown():

        t = rospy.Time.now().secs

        msg1.data = 200 * np.pi / 180 * np.sin(2 * np.pi * t / 10)
        msg2.data =  50 * np.pi / 180 * np.sin(2 * np.pi * t / 12)
        msg3.data = -80 * np.pi / 180 * np.sin(2 * np.pi * t / 15)
        msg4.data =  60 * np.pi / 180 * np.sin(2 * np.pi * t / 11)

        pub1.publish(msg1)
        pub2.publish(msg2)
        pub3.publish(msg3)
        pub4.publish(msg4)


if __name__ == '__main__':
    try:
        joint_pub()
    except rospy.ROSInterruptException:
        pass