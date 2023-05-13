#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import time 
import numpy as np
from tf import transformations


# def update_euler(self):
#         q = self.q_auv
#         q = np.array([q.x, q.y, q.z, q.w])
        
#         theta_x = transformations.euler_from_quaternion(q, 'rxyz')[0]
#         theta_y = transformations.euler_from_quaternion(q, 'ryxz')[0]
#         theta_z = transformations.euler_from_quaternion(q, 'rzyx')[0]

#         angles = np.array([theta_x, theta_y, theta_z])*DEG_PER_RAD

#         for i in range(3):
#             if angles[i] - self.euler[i] > ANGLE_CHANGE_TOL:
#                 self.euler[i] = angles[i] - 360
#             elif self.euler[i] - angles[i] > ANGLE_CHANGE_TOL:
#                 self.euler[i] = angles[i] + 360
#             else:
#                 self.euler[i] = angles[i]


def callback_imu(data):
     pub_state_x.publish(data)
     pub_state_y.publish(data)
     pub_state_z.publish(data)
     pub_state_theta_x.publish(data)
     pub_state_theta_y.publish(data)
     pub_state_theta_z.publish(data)


if __name__ == '__main__':

    rospy.init_node('thrusters')
    pubt0 = rospy.Publisher('/model/clarke/joint/thruster0_joint/cmd_pos', Float64, queue_size=1)
    pubt1 = rospy.Publisher('/model/clarke/joint/thruster1_joint/cmd_pos', Float64, queue_size=1) 
    pubt2 = rospy.Publisher('/model/clarke/joint/thruster2_joint/cmd_pos', Float64, queue_size=1)
    pubt3 = rospy.Publisher('/model/clarke/joint/thruster3_joint/cmd_pos', Float64, queue_size=1)
    pubt4 = rospy.Publisher('/model/clarke/joint/thruster4_joint/cmd_pos', Float64, queue_size=1)
    pubt5 = rospy.Publisher('/model/clarke/joint/thruster5_joint/cmd_pos', Float64, queue_size=1)
    pubt6 = rospy.Publisher('/model/clarke/joint/thruster6_joint/cmd_pos', Float64, queue_size=1)
    pubt7 = rospy.Publisher('/model/clarke/joint/thruster7_joint/cmd_pos', Float64, queue_size=1)
    pub_state_x = rospy.Publisher('/state_x', Float64, queue_size=1)
    pub_state_y = rospy.Publisher('/state_y', Float64, queue_size=1)
    pub_state_z = rospy.Publisher('/state_z', Float64, queue_size=1)
    pub_state_theta_x = rospy.Publisher('/state_theta_x', Float64, queue_size=1)
    pub_state_theta_y = rospy.Publisher('/state_theta_y', Float64, queue_size=1)
    pub_state_theta_z = rospy.Publisher('/state_theta_z', Float64, queue_size=1)

    sub_imu = rospy.Subscriber('/imu', Imu, callback_imu)
    # pub_depth_cam = rospy.Publisher('/[Front camera topic]', Sensor_msgs/msg/Image, queue_size=1)
    # sub_depth_cam = rospy.Subscriber('/depth_cam', Sensor_msgs/msg/Image, queue_size=1)
    while True:
        pubt1.publish(50.0)
        pubt0.publish(50.0)

        #pubt4.publish(50.0)
        #pubt5.publish(50.0)
        #pubt6.publish(50.0)
        #pubt7.publish(50.0)

        time.sleep(1)    
