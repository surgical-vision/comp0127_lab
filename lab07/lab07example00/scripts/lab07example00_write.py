#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int32, String
import rospkg

def lab07_example00_write():

    rospy.init_node('lab07_example00_write')

    rospack = rospkg.RosPack()

    path = rospack.get_path('lab07_example00')
    bag = rosbag.Bag(path + '/../example00_data.bag', 'w')

    try:
		str = String()
		str.data = 'foo'

		i = Int32()
		i.data = 42

		bag.write('chatter', str)
		bag.write('numbers', i)
    finally:
        bag.close()

if __name__ == '__main__':
    try:
        lab07_example00_write()
    except rospy.ROSInterruptException:
        pass