#!/usr/bin/env python3

import rospy
import numpy as np

from auv_msgs.msg import ThrusterForces, DeadReckonReport
from geometry_msgs.msg import PoseArray, Vector3, Quaternion 
from sbg_driver.msg import SbgImuData, SbgEkfQuat
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf import transformations


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
    
    dr_msg = DeadReckonReport()
    dr_msg.x = clarke_position.x
    dr_msg.y = clarke_position.y
    dr_msg.z = clarke_position.z

    depth = clarke_position.z # TODO - check with reference to which datum

    # TODO - specify roll/pitch/yaw for dvl
    pub_dvl_deadreckon.publish(dr_msg)
    pub_depth_sensor.publish(depth)


def cb_sim_imu(data):
    # TODO - attach imu link to preserve orientation
    gyro = Vector3(-data.angular_velocity.z, -data.angular_velocity.y, -data.angular_velocity.x)
    pub_imu_angular_vel.publish(gyro)
    
    sbg_quat = SbgEkfQuat()
    quat = Quaternion(-data.orientation.z, -data.orientation.y, -data.orientation.x, data.orientation.w) # TODO - incorrect?
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
    pub_depth_sensor = rospy.Publisher('/depth', Float64, queue_size=1)
    pub_imu_quat = rospy.Publisher('sbg/ekf_quat', SbgEkfQuat, queue_size=1)
    # TODO - state_estimation should be responsible for this - SbgImuData
    pub_imu_angular_vel = rospy.Publisher('angular_velocity', Vector3, queue_size=1)
    pub_dvl_deadreckon = rospy.Publisher('dead_reckon_report', DeadReckonReport, queue_size=1)

    rospy.Subscriber('/imu', Imu, cb_sim_imu)
    rospy.Subscriber('/world/quali/dynamic_pose/info', PoseArray, cb_sim_pose)

    rospy.spin()
