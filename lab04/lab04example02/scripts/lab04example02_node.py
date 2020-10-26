#!/usr/bin/env python

import rospy
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('robotis_tf_listener_node')

    tfBuffer = tf2_ros.Buffer() #Initialise tf
    listener = tf2_ros.TransformListener(tfBuffer) #Subscriber for tf topic

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

        try:
            T = tfBuffer.lookup_transform('world', 'link6', rospy.Time(0)) #Look for a transformation that links the frame "world" and "link6" together
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
            rate.sleep()
            continue

        print T

        rate.sleep()
