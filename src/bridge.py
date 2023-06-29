#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Wrench, PoseArray, Pose, Vector3, Quaternion 
from sbg_driver.msg import SbgImuData, SbgEkfQuat
import numpy as np
from tf import transformations
import math


DEG_PER_RAD = 180/np.pi
ANGLE_CHANGE_TOL = 90 
euler = np.array([0.0, 0.0, 0.0])

d = 0.2222 #m
hypot = math.sqrt(math.pow(0.5588/2,2) + math.pow(0.2222/2,2)) #m
D_2 = 0.5588 #m

# Thursters positions
#     3
#  5         6
#  1         2
#  7         8
#         4  

T = np.matrix(
        [[-1.,  -1.,  0.,    0.,    0.,     0.,     0.,     0.],
        [  0.,   0.,  1.,   -1.,    0.,     0.,     0.,     0.],
        [  0.,   0.,  0.,    0.,   -1.,    -1.,    -1.,    -1.],
        [  0.,   0.,  0.,    0.,   -d/2,    d/2,   -d/2,    d/2],
        [  0.,   0.,  0.,    0.,   D_2/2,   D_2/2, -D_2/2, -D_2/2],
        [  d/2, -d/2, hypot, hypot, 0.,     0.,     0.,     0.]]
        )

# forces produced by T200 thruster at 14V (N)
THRUST_LIMIT = 1.0  # Limit thruster speed while dry-testing
MAX_FWD_FORCE = 4.52*9.81*THRUST_LIMIT
MAX_BKWD_FORCE = -3.52*9.81*THRUST_LIMIT

T_inv = np.linalg.pinv(T)

euler = np.array([0.0, 0.0, 0.0])


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

    pubt1.publish(converted_w[0])
    pubt2.publish(converted_w[1])
    pubt3.publish(converted_w[2])
    pubt4.publish(converted_w[3])
    pubt5.publish(converted_w[4])
    pubt6.publish(converted_w[5])
    pubt7.publish(converted_w[6])
    pubt8.publish(converted_w[7])

def callback_pose(data):
    global euler
    
    clarke_poses = data.poses[0]
    clarke_position = clarke_poses.position
    clarke_orientation = clarke_poses.orientation
    pub_state_x.publish(clarke_position.x + 3)
    pub_state_y.publish(clarke_position.y - 0.5)
    pub_state_z.publish(clarke_position.z)
    
    pose = Pose()
    pose.position.x = clarke_position.x + 3
    pose.position.y = clarke_position.y - 0.5
    pose.position.z = clarke_position.z
    pose.orientation = clarke_orientation
    pub_pose.publish(pose)

    q = np.array([clarke_orientation.x, clarke_orientation.y, clarke_orientation.z, clarke_orientation.w])
    conversion = transformations.euler_from_quaternion(q, 'rxyz')
    theta_x = conversion[0]
    theta_y = conversion[1]
    theta_z = conversion[2]

    angles = np.array([theta_x, theta_y, theta_z])*DEG_PER_RAD

    for i in range(3):
            if angles[i] - euler[i] > ANGLE_CHANGE_TOL:
                euler[i] = angles[i] - 360
            elif euler[i] - angles[i] > ANGLE_CHANGE_TOL:
                euler[i] = angles[i] + 360
            else:
                euler[i] = angles[i]

    pub_state_theta_x.publish(euler[0])
    pub_state_theta_y.publish(euler[1])
    pub_state_theta_z.publish(euler[2])
    
def callback_imu(data):
    imu_data_msg = SbgImuData()
    gyro = Vector3(-data.angular_velocity.z, -data.angular_velocity.y, -data.angular_velocity.x)
    imu_data_msg.gyro = gyro
    imu_data_pub.publish(imu_data_msg)
    
    imu_ekf_quat_msg = SbgEkfQuat()
    quat = Quaternion(-data.orientation.z, -data.orientation.y, -data.orientation.x, data.orientation.w)
    imu_ekf_quat_msg.quaternion = quat
    imu_quat_pub.publish(imu_ekf_quat_msg)
    

if __name__ == '__main__':

    rospy.init_node('bridge')

    pub_state_x = rospy.Publisher('state_x', Float64, queue_size=1)
    pub_state_y = rospy.Publisher('state_y', Float64, queue_size=1)
    pub_state_z = rospy.Publisher('state_z', Float64, queue_size=1)
    pub_state_theta_x = rospy.Publisher('state_theta_x', Float64, queue_size=1)
    pub_state_theta_y = rospy.Publisher('state_theta_y', Float64, queue_size=1)
    pub_state_theta_z = rospy.Publisher('state_theta_z', Float64, queue_size=1)
    pub_pose = rospy.Publisher('pose', Pose, queue_size=1)

    pub_y_pid = rospy.Publisher('y_setpoint', Float64, queue_size=1)
    pub_x_pid = rospy.Publisher('x_setpoint', Float64, queue_size=1)
    pub_z_pid = rospy.Publisher('z_setpoint', Float64, queue_size=1)
    pub_theta_x_pid = rospy.Publisher('theta_x_setpoint', Float64, queue_size=1)
    pub_theta_y_pid = rospy.Publisher('theta_y_setpoint', Float64, queue_size=1)
    pub_theta_z_pid = rospy.Publisher('theta_z_setpoint', Float64, queue_size=1)    

    sub_pose = rospy.Subscriber('/world/quali/dynamic_pose/info', PoseArray, callback_pose)
    
    
    imu_data_pub = rospy.Publisher('/sbg/imu_data', SbgImuData, queue_size=1)
    imu_quat_pub = rospy.Publisher('sbg/ekf_quat', SbgEkfQuat, queue_size=1)
    
    imu_sub = rospy.Subscriber('/imu', Imu, callback_imu)

    pubt1 = rospy.Publisher('/model/clarke/joint/thruster1_joint/cmd_pos', Float64, queue_size=1)
    pubt2 = rospy.Publisher('/model/clarke/joint/thruster2_joint/cmd_pos', Float64, queue_size=1) 
    pubt3 = rospy.Publisher('/model/clarke/joint/thruster3_joint/cmd_pos', Float64, queue_size=1)
    pubt4 = rospy.Publisher('/model/clarke/joint/thruster4_joint/cmd_pos', Float64, queue_size=1)
    pubt5 = rospy.Publisher('/model/clarke/joint/thruster5_joint/cmd_pos', Float64, queue_size=1)
    pubt6 = rospy.Publisher('/model/clarke/joint/thruster6_joint/cmd_pos', Float64, queue_size=1)
    pubt7 = rospy.Publisher('/model/clarke/joint/thruster7_joint/cmd_pos', Float64, queue_size=1)
    pubt8 = rospy.Publisher('/model/clarke/joint/thruster8_joint/cmd_pos', Float64, queue_size=1)
    
    sub_effort = rospy.Subscriber('/effort', Wrench, callback_thrusters)    

    
    rospy.spin()
    
    # rate = rospy.Rate(10)

    # while True:
        # pub_z_pid.publish(0.0)
        # pub_x_pid.publish(0.0)
        # pub_y_pid.publish(0.0)

        # pub_theta_z_pid.publish(0.0)
        # pub_theta_x_pid.publish(0.0)
        # pub_theta_y_pid.publish(0.0)
        
        # pubt1.publish(10.0)
        # pubt2.publish(10.0)
        
        # pubt3.publish(1.0)
        # pubt4.publish(1.0)
        
        # pubt5.publish(-20.0)
        # pubt6.publish(-20.0)
        # pubt7.publish(-20.0)
        # pubt8.publish(-20.0)
        
        # rate.sleep()

    
    

        
           
