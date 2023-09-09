#!/usr/bin/env python3

import rospy
import numpy as np

from auv_msgs.msg import ThrusterForces, DeadReckonReport
from geometry_msgs.msg import PoseArray, Vector3, Quaternion, Pose
from sbg_driver.msg import SbgImuData, SbgEkfQuat, SbgImuStatus
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, UInt32, Float32
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


def cb_sim_dvl_depth(data):
    dvl_sim = data.poses[2]
    
    dvl_sim_position = dvl_sim.position
    dvl_sim_orientation = dvl_sim.orientation

    dvl_msg = DeadReckonReport()
    
    dvl_msg.x = dvl_sim_position.x
    dvl_msg.y = dvl_sim_position.y
    dvl_msg.z = dvl_sim_position.z
    
    dvl_msg.std = 0.0
    
    dvl_msg.status = 1
    
    q = np.array([dvl_sim_orientation.x, dvl_sim_orientation.y, dvl_sim_orientation.z, dvl_sim_orientation.w])
    angles = np.asarray(tf.transformations.euler_from_quaternion(q, 'rxyz')) * DEG_PER_RAD

    dvl_msg.roll = angles[0]
    dvl_msg.pitch = angles[1]
    dvl_msg.yaw = angles[2]
    
    pub_dvl_sensor.publish(dvl_msg)
    
    clarke_msg = data.poses[0]

    depth_msg = clarke_msg.position.z
    pub_depth_sensor.publish(depth_msg)

def cb_sim_imu(data):
    sbg_quat_msg = SbgEkfQuat()
    sbg_quat_msg.quaternion = Quaternion(-data.orientation.z, data.orientation.y, data.orientation.x, data.orientation.w)    
    
    sbg_data_msg = SbgImuData()
    sbg_data_msg.gyro = Vector3(-data.angular_velocity.z, data.angular_velocity.y, data.angular_velocity.x)

    pub_imu_quat_sensor.publish(sbg_quat_msg)
    pub_imu_data_sensor.publish(sbg_data_msg)

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
    
    pub_dvl_sensor = rospy.Publisher('dead_reckon_report', DeadReckonReport, queue_size=1)

    pub_depth_sensor = rospy.Publisher('depth', Float64, queue_size=1)

    pub_imu_quat_sensor = rospy.Publisher('sbg/ekf_quat', SbgEkfQuat, queue_size=1)
    pub_imu_data_sensor = rospy.Publisher('sbg/imu_data', SbgImuData, queue_size=1)

    rospy.Subscriber('/imu', Imu, cb_sim_imu)
    rospy.Subscriber('/world/quali/dynamic_pose/info', PoseArray, cb_sim_dvl_depth)

    rospy.spin()
