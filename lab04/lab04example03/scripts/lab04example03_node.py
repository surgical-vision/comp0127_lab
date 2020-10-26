#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import PyKDL as kdl


chain=kdl.Chain()



a = [0.0, 0.265699, 0.03, 0.0, 0.0, 0.0]
alpha = [-pi/2, 0.0, -pi/2, -pi/2, -pi/2, 0.0]
d = [0.159, 0.0, 0.0, 0.258, 0.0, -0.123]
theta = [0.0, -pi/2+np.arctan(0.03/0.264), -np.arctan(0.03/0.264), 0.0, 0.0, 0.0]
name_link = ['kdl_link_1', 'kdl_link_2', 'kdl_link_3', 'kdl_link_4', 'kdl_link_5', 'kdl_link_6']


chain=kdl.Chain()

for p_a, p_alpha, p_d, p_theta in zip(a, alpha, d, theta):
    chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ), kdl.Frame().DH(a=p_a,alpha=p_alpha, d=p_d, theta=p_theta)))

fk=kdl.ChainFkSolverPos_recursive(chain)

##Declaring a transformBroadcaster like this is not a very optimal way.
##The very optimal way is to create a "class" for it.
##We will ask you to do this in the next Coursework. :)

br = tf2_ros.TransformBroadcaster()


def fkine_kdl(joint_msg):

    # convert joint_msg to a kdl joint array    
    joint_positions = kdl.JntArray(chain.getNrOfJoints())

    for i in range(chain.getNrOfJoints()):
        if (i==4 or i==5):
            joint_positions[i] = - joint_msg.position[i]
        else:
            joint_positions[i] = joint_msg.position[i]

    transform = TransformStamped()
    frame_t = kdl.Frame()

    for i in range(chain.getNrOfJoints()):
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'world'
        transform.child_frame_id = name_link[i]
        #solve foraward kinematics up to link i
        fk.JntToCart(joint_positions, frame_t, i+1)

        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'world'
        transform.child_frame_id = name_link[i]

        transform.transform.translation.x = frame_t.p[0]
        transform.transform.translation.y = frame_t.p[1]
        transform.transform.translation.z = frame_t.p[2]
        transform.transform.rotation = Quaternion(*frame_t.M.GetQuaternion())

        br.sendTransform(transform)


def fkine_main():
    rospy.init_node('fkine_kdl_node')

    sub = rospy.Subscriber('/joint_states', JointState, fkine_kdl)
    
    rate = rospy.Rate(10)
    rospy.spin()
    rate.sleep()


if __name__ == '__main__':
    try:
        fkine_main()
    except rospy.ROSInterruptException:
        pass