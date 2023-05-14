#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Wrench
import time 
import numpy as np
from tf import transformations


d = 0.224 #m
D_1 = 0.895 #m
D_2 = 0.778 #m

MAX_FWD_FORCE = 4.52*9.81*0.5 
MAX_BKWD_FORCE = -3.52*9.81*0.5 

T = np.matrix(
        [[1., 1., 0., 0., 0., 0., 0., 0.],
        [0., 0., 1., -1., 0., 0., 0., 0.],
        [0., 0., 0., 0., -1., -1., -1., -1.],
        [0., 0., 0., 0., -d/2, d/2, d/2, -d/2],
        [0., 0., 0., 0., -D_2/2, -D_2/2, D_2/2, D_2/2],
        [-d/2, d/2, D_1/2, D_1/2, 0., 0., 0., 0.]]
        )

T_inv = np.linalg.pinv(T)


def callback_thrusters(data):
    a = np.array(
            [[data.force.x],
            [data.force.y],
            [data.force.z],
            [data.torque.x],
            [data.torque.y],
            [data.torque.z]]
            )

    converted_w = np.matmul(T_inv, a) 

    pubt0.publish(converted_w[0])
    pubt1.publish(converted_w[1])
    pubt2.publish(converted_w[2])
    pubt3.publish(converted_w[3])
    pubt4.publish(converted_w[4])
    pubt5.publish(converted_w[5])
    pubt6.publish(converted_w[6])
    pubt7.publish(converted_w[7])

def callback_imu_dvl(data):
    p = data.orientation
    q = np.array([p.x, p.y, p.z, p.w])
    theta_x = transformations.euler_from_quaternion(q, 'rxyz')[0]
    theta_y = transformations.euler_from_quaternion(q, 'rxyz')[0]
    theta_z = transformations.euler_from_quaternion(q, 'rxyz')[0]
    pub_state_x.publish(p.x)
    pub_state_y.publish(p.y)
    pub_state_z.publish(p.z)
    pub_state_theta_x.publish(theta_x)
    pub_state_theta_y.publish(theta_y)
    pub_state_theta_z.publish(theta_z)


if __name__ == '__main__':

    rospy.init_node('thrusters')

    # IMU + DVL
    pub_state_x = rospy.Publisher('/state_x', Float64, queue_size=1)
    pub_state_y = rospy.Publisher('/state_y', Float64, queue_size=1)
    pub_state_z = rospy.Publisher('/state_z', Float64, queue_size=1)
    pub_state_theta_x = rospy.Publisher('/state_theta_x', Float64, queue_size=1)
    pub_state_theta_y = rospy.Publisher('/state_theta_y', Float64, queue_size=1)
    pub_state_theta_z = rospy.Publisher('/state_theta_z', Float64, queue_size=1)

    # pub_dvl = rospy.Publisher('', pass, queue_size=1)

    sub_imu = rospy.Subscriber('/imu', Imu, callback_imu_dvl)

    # Thrusters
    pubt0 = rospy.Publisher('/model/clarke/joint/thruster0_joint/cmd_pos', Float64, queue_size=1)
    pubt1 = rospy.Publisher('/model/clarke/joint/thruster1_joint/cmd_pos', Float64, queue_size=1) 
    pubt2 = rospy.Publisher('/model/clarke/joint/thruster2_joint/cmd_pos', Float64, queue_size=1)
    pubt3 = rospy.Publisher('/model/clarke/joint/thruster3_joint/cmd_pos', Float64, queue_size=1)
    pubt4 = rospy.Publisher('/model/clarke/joint/thruster4_joint/cmd_pos', Float64, queue_size=1)
    pubt5 = rospy.Publisher('/model/clarke/joint/thruster5_joint/cmd_pos', Float64, queue_size=1)
    pubt6 = rospy.Publisher('/model/clarke/joint/thruster6_joint/cmd_pos', Float64, queue_size=1)
    pubt7 = rospy.Publisher('/model/clarke/joint/thruster7_joint/cmd_pos', Float64, queue_size=1)
    
    sub_effort = rospy.Subscriber('/effort', Wrench, callback_thrusters)

    while True:
        pubt0.publish(50.0)
        pubt1.publish(50.0)
        rospy.sleep(1)
        pubt4.publish(100.0)
        pubt5.publish(100.0)
        pubt6.publish(100.0)
        pubt7.publish(100.0)
        
           
