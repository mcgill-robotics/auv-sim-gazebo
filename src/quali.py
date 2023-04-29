#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':

    rospy.init_node('thruster0')
    pub = rospy.Publisher('/test', String, queue_size=50)
    while True:
        pub.publish("-30.0")
