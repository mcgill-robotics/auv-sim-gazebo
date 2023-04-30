#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':

    rospy.init_node('thruster0')
    pub = rospy.Publisher('/model/clarke/joint/thruster0_joint/cmd_pos', Float64, queue_size=50)
    while True:
        pub.publish(-30.0)
