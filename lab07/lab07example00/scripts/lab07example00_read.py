#!/usr/bin/env python
import rospy
import rosbag
import rospkg
from std_msgs.msg import Int32, String

def lab07_example00_read():

    rospy.init_node('lab07_example00_read')

    rospack = rospkg.RosPack()

    path = rospack.get_path('lab07_example00')
    bag = rosbag.Bag(path + '/../example00_data.bag')

    for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
		print msg

    bag.close()

if __name__ == '__main__':
    try:
        lab07_example00_read()
    except rospy.ROSInterruptException:
        pass