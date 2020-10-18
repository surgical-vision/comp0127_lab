#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64

joint1 = [1.7768, 0.2057, 2.8906, 3.3794, 2.9092, 2.3643, 1.2801, 2.2591, 1.7753, 0.9860]
joint2 = [1.7252, 0.0778, 0.6767, 0.1128, 0.2373, 2.0121, 1.6978, 0.7748, 2.3218, 0.0842]
joint3 = [-1.2207, -1.9312, -1.8746, -1.0249, -0.9458, -1.4789, -1.2553, -1.2713, -1.5904, -1.8198]
joint4 = [0.9635, 1.3726, 1.2867, 0.5676, 0.4154, 1.7396, 1.3501, 1.1882, 1.0430, 0.7813]

def move_youbot():
    rospy.init_node('trajectory_generator')

    j1_pub = rospy.Publisher('/EffortJointInterface_J1_controller/command', Float64, queue_size=10)
    j2_pub = rospy.Publisher('/EffortJointInterface_J2_controller/command', Float64, queue_size=10)
    j3_pub = rospy.Publisher('/EffortJointInterface_J3_controller/command', Float64, queue_size=10)
    j4_pub = rospy.Publisher('/EffortJointInterface_J4_controller/command', Float64, queue_size=10)

    t = 0

    msg1 = Float64()
    msg2 = Float64()
    msg3 = Float64()
    msg4 = Float64()

    while not rospy.is_shutdown():
        msg1.data = joint1[t]
        msg2.data = joint2[t]
        msg3.data = joint3[t]
        msg4.data = joint4[t]

        j1_pub.publish(msg1)
        j2_pub.publish(msg2)
        j3_pub.publish(msg3)
        j4_pub.publish(msg4)

        if (t == 9):
            t = 0
        else:
            t = t + 1
        time.sleep(2)

if __name__ == '__main__':
    try:
        move_youbot()
    except rospy.ROSInterruptException:
        pass
