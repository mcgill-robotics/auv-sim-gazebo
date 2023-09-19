#!/usr/bin/env python3

import rospy
import numpy as np

from auv_msgs.msg import ThrusterForces, DeadReckonReport
from geometry_msgs.msg import PoseArray, Vector3, Quaternion
from sbg_driver.msg import SbgImuData, SbgEkfQuat
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf
import quaternion


DEG_PER_RAD = (180 / np.pi)

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
    dvl_sim = data.poses[0]
    
    # DVL FRAME: NWU

    #DVL POSITION
    clarke_sim_position = dvl_sim.position
    auv_np_position = np.array([clarke_sim_position.x, clarke_sim_position.y, clarke_sim_position.z])
    dvl_auv_offset_rotated = quaternion.rotate_vectors(q_gazebo_dvl_NED.inverse(), np.array([auv_dvl_offset_x, auv_dvl_offset_y, auv_dvl_offset_z])) 
    dvl_position = quaternion.rotate_vectors(q_gazebo_dvl_NED.inverse(), auv_np_position) + dvl_auv_offset_rotated

    #DVL QUATERNION
    clarke_sim_orientation = dvl_sim.orientation
    clarke_np_quat = np.quaternion(clarke_sim_orientation.w, clarke_sim_orientation.x, clarke_sim_orientation.y, clarke_sim_orientation.z)
    dvl_quat = q_gazebo_dvl_NED * clarke_np_quat
    dvl_quat = dvl_quat * q_gazebo_dvl_NED.inverse()
    angles = np.asarray(tf.transformations.euler_from_quaternion([dvl_quat.x, dvl_quat.y, dvl_quat.z, dvl_quat.w], 'rxyz')) * DEG_PER_RAD

    dvl_msg = DeadReckonReport()

    dvl_msg.x = dvl_position[0]
    dvl_msg.y = dvl_position[1]
    dvl_msg.z = dvl_position[2]
    dvl_msg.std = 0.0
    dvl_msg.status = 1
    dvl_msg.roll = angles[0]
    dvl_msg.pitch = angles[1]
    dvl_msg.yaw = angles[2]
    pub_dvl_sensor.publish(dvl_msg)
    
    # DEPTH FRAME: NWU
    # DEPTH SENSOR
    clarke_msg = data.poses[0]
    depth_msg = clarke_msg.position.z
    pub_depth_sensor.publish(depth_msg)

def cb_sim_imu(data):
    # IMU FRAME:
    #   x: down
    #   y: east
    #   z: south
    sbg_quat_msg = SbgEkfQuat()
    
    clarke_np_quat = np.quaternion(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)

    imu_quat = q_gazebo_imu_NED * clarke_np_quat * q_gazebo_imu_NED.inverse()
    angles = np.asarray(tf.transformations.euler_from_quaternion([imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w], 'rxyz'))
    imu_quat_tf = tf.transformations.quaternion_from_euler(-angles[0], angles[1], -angles[2])
    imu_quat = np.quaternion(imu_quat_tf[3], imu_quat_tf[0], imu_quat_tf[1], imu_quat_tf[2])
    sbg_quat_msg.quaternion = Quaternion(imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w) 
    
    sbg_data_msg = SbgImuData()
    
    ang_vel = quaternion.rotate_vectors(q_gazebo_imu_NED.inverse(), np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]))
    sbg_data_msg.gyro = Vector3(ang_vel[0], ang_vel[1], ang_vel[2])

    pub_imu_quat_sensor.publish(sbg_quat_msg)
    pub_imu_data_sensor.publish(sbg_data_msg)

if __name__ == '__main__':

    rospy.init_node('bridge')
    
    q_dvl_auv_w = rospy.get_param("~q_dvl_auv_w")
    q_dvl_auv_x = rospy.get_param("~q_dvl_auv_x")
    q_dvl_auv_y = rospy.get_param("~q_dvl_auv_y")
    q_dvl_auv_z = rospy.get_param("~q_dvl_auv_z")
    
    auv_dvl_offset_x = rospy.get_param("~auv_dvl_offset_x")
    auv_dvl_offset_y = rospy.get_param("~auv_dvl_offset_y")
    auv_dvl_offset_z = rospy.get_param("~auv_dvl_offset_z")
    
    q_imu_auv_w = rospy.get_param("~q_imu_auv_w")
    q_imu_auv_x = rospy.get_param("~q_imu_auv_x")
    q_imu_auv_y = rospy.get_param("~q_imu_auv_y")
    q_imu_auv_z = rospy.get_param("~q_imu_auv_z")
    
    # REFERENCE FRAME DEFINITIONS
    
    q_NWU_NED = np.quaternion(0, 1, 0, 0)

    q_imu_auv = np.quaternion(q_imu_auv_w, q_imu_auv_x, q_imu_auv_y, q_imu_auv_z)
    q_gazebo_imu_NWU = np.quaternion(0, -0.7071068, 0, 0.7071068)
    q_gazebo_imu_NED = q_gazebo_imu_NWU * q_imu_auv * q_NWU_NED

    q_dvl_auv = np.quaternion(q_dvl_auv_w, q_dvl_auv_x, q_dvl_auv_y, q_dvl_auv_z)
    q_gazebo_dvl_NED = q_dvl_auv * q_NWU_NED

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
