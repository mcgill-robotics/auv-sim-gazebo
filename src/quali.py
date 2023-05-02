#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench
import numpy as np
from auv_msgs.msg import ThrusterForces


d = 0.224 #m
D_1 = 0.895 #m
D_2 = 0.778 #m

#Matrix representation of the system of equations representing the thrust to wrench conversion
#Ex: Force_X = (1)Surge_Port_Thruster + (1)Surge_Starboard_Thrust
T = np.matrix(
        [[1., 1., 0., 0., 0., 0., 0., 0.],
        [0., 0., 1., -1., 0., 0., 0., 0.],
        [0., 0., 0., 0., -1., -1., -1., -1.],
        [0., 0., 0., 0., -d/2, d/2, d/2, -d/2],
        [0., 0., 0., 0., -D_2/2, -D_2/2, D_2/2, D_2/2],
        [-d/2, d/2, D_1/2, D_1/2, 0., 0., 0., 0.]]
        )

#matrix transformation wrench -> thrust 
T_inv = np.linalg.pinv(T) 
"""--------------------------------------------------"""
rospy.sleep(7.0)


def wrench_to_thrust(w):
    '''
    A callback function that maps a Wrench into a force produced by T200 thruster at 14V (N)
    '''
    a = np.array(
            [[w.force.x],
            [w.force.y],
            [w.force.z],
            [w.torque.x],
            [w.torque.y],
            [w.torque.z]]
            )

    converted_w = np.matmul(T_inv, a) 
     
    pubt0.publish(Float64(converted_w[0]))
    pubt1.publish(Float64(converted_w[1]))
    pubt2.publish(Float64(converted_w[2]) * -1.0)
    pubt3.publish(Float64(converted_w[3]))
    pubt4.publish(Float64(converted_w[4]))
    pubt5.publish(Float64(converted_w[5]))
    pubt6.publish(Float64(converted_w[6]))
    pubt7.publish(Float64(converted_w[7]))

    """
    tf = ThrusterForces()
    tf.SURGE_PORT = converted_w[0]
    tf.SURGE_STAR = converted_w[1]
    tf.SWAY_BOW = converted_w[2]
    tf.SWAY_STERN = converted_w[3]
    tf.HEAVE_BOW_PORT = converted_w[4]
    tf.HEAVE_BOW_STAR = converted_w[5]
    tf.HEAVE_STERN_STAR = converted_w[6]
    tf.HEAVE_STERN_PORT = converted_w[7]
    """

    
    


if __name__ == '__main__':

    rospy.init_node('thrusters')
    subt = rospy.Subscriber('/effort', Wrench, wrench_to_thrust)
    pubt0 = rospy.Publisher('/model/clarke/joint/thruster0_joint/cmd_pos', Float64, queue_size=50)
    pubt1 = rospy.Publisher('/model/clarke/joint/thruster1_joint/cmd_pos', Float64, queue_size=50)
    pubt2 = rospy.Publisher('/model/clarke/joint/thruster2_joint/cmd_pos', Float64, queue_size=50)
    pubt3 = rospy.Publisher('/model/clarke/joint/thruster3_joint/cmd_pos', Float64, queue_size=50)
    pubt4 = rospy.Publisher('/model/clarke/joint/thruster4_joint/cmd_pos', Float64, queue_size=50)
    pubt5 = rospy.Publisher('/model/clarke/joint/thruster5_joint/cmd_pos', Float64, queue_size=50)
    pubt6 = rospy.Publisher('/model/clarke/joint/thruster6_joint/cmd_pos', Float64, queue_size=50)
    pubt7 = rospy.Publisher('/model/clarke/joint/thruster7_joint/cmd_pos', Float64, queue_size=50)
    
