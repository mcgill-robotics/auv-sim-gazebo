#!/usr/bin/env python3

import rospy
import numpy as np

from auv_msgs.msg import ThrusterForces, DeadReckonReport
from geometry_msgs.msg import PoseArray, Vector3, Quaternion, Pose
from sbg_driver.msg import SbgImuData, SbgEkfQuat
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf


DEG_PER_RAD = 180/np.pi
ANGLE_CHANGE_TOL = 90 
euler = np.array([0.0, 0.0, 0.0])


def cb_thrusters(data):
    # TODO - this needs to be N/N*m not us - publish from propulsion
    pub_t1.publish(data.SURGE_PORT)
    pub_t2.publish(data.SURGE_STAR)
    pub_t3.publish(data.SWAY_BOW)
    pub_t4.publish(data.SWAY_STERN)
    pub_t5.publish(data.HEAVE_BOW_PORT)
    pub_t6.publish(data.HEAVE_BOW_STAR)
    pub_t7.publish(data.HEAVE_STERN_STAR)
    pub_t8.publish(data.HEAVE_STERN_PORT)


def cb_sim_pose(data):
    clarke_poses = data.poses[0]
    clarke_position = clarke_poses.position
    clarke_orientation = clarke_poses.orientation
    pub_state_x.publish(clarke_position.x)
    pub_state_y.publish(clarke_position.y)
    pub_state_z.publish(clarke_position.z)

    pose = Pose()
    pose.position.x = clarke_position.x
    pose.position.y = clarke_position.y
    pose.position.z = clarke_position.z
    pose.orientation = clarke_orientation
    pub_pose.publish(pose)

    q = np.array([clarke_orientation.x, clarke_orientation.y, clarke_orientation.z, clarke_orientation.w])
    conversion = tf.transformations.euler_from_quaternion(q, 'rxyz')
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

def cb_sim_imu(data):
    # TODO - attach imu link to preserve orientation
    gyro = Vector3(-data.angular_velocity.z, -data.angular_velocity.y, -data.angular_velocity.x)
    pub_imu_angular_vel.publish(gyro)
    
    sbg_quat = SbgEkfQuat()
    quat = Quaternion(-data.orientation.z, data.orientation.y, data.orientation.x, data.orientation.w) # TODO - incorrect?
    sbg_quat.quaternion = quat
    pub_imu_quat.publish(sbg_quat)
    

if __name__ == '__main__':

    rospy.init_node('bridge')

    # simulate propulsion thrusters
    pub_t1 = rospy.Publisher('/model/clarke/joint/thruster1_joint/cmd_pos', Float64, queue_size=1)
    pub_t2 = rospy.Publisher('/model/clarke/joint/thruster2_joint/cmd_pos', Float64, queue_size=1) 
    pub_t3 = rospy.Publisher('/model/clarke/joint/thruster3_joint/cmd_pos', Float64, queue_size=1)
    pub_t4 = rospy.Publisher('/model/clarke/joint/thruster4_joint/cmd_pos', Float64, queue_size=1)
    pub_t5 = rospy.Publisher('/model/clarke/joint/thruster5_joint/cmd_pos', Float64, queue_size=1)
    pub_t6 = rospy.Publisher('/model/clarke/joint/thruster6_joint/cmd_pos', Float64, queue_size=1)
    pub_t7 = rospy.Publisher('/model/clarke/joint/thruster7_joint/cmd_pos', Float64, queue_size=1)
    pub_t8 = rospy.Publisher('/model/clarke/joint/thruster8_joint/cmd_pos', Float64, queue_size=1)
    rospy.Subscriber('propulsion/thruster_forces', ThrusterForces, cb_thrusters)

    # simulate state_estimation sensors
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

    pub_imu_quat = rospy.Publisher('sbg/ekf_quat', SbgEkfQuat, queue_size=1)
    pub_imu_angular_vel = rospy.Publisher('angular_velocity', Vector3, queue_size=1)

    rospy.Subscriber('/imu', Imu, cb_sim_imu)
    rospy.Subscriber('/world/quali/dynamic_pose/info', PoseArray, cb_sim_pose)

    rospy.spin()
