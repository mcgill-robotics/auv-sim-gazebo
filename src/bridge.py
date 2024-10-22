#!/usr/bin/env python3

import rospy
import numpy as np

from auv_msgs.msg import ThrusterForces, DeadReckonReport
from geometry_msgs.msg import PoseArray, Vector3, Quaternion
from sbg_driver.msg import SbgImuData, SbgEkfQuat
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf import transformations
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
    
    pose_NWU_auv = data.poses[0]
    # Convert to an NED pose for testing sake

    #DVL POSITION
    position_NWU_auv = np.array([pose_NWU_auv.position.x, pose_NWU_auv.position.y, pose_NWU_auv.position.z])
    position_auv_dvlref = quaternion.rotate_vectors(q_NWU_dvlref.inverse(), position_NWU_auv)
    dvl_offset_NWU = quaternion.rotate_vectors(q_NWU_dvlref, np.array([auv_dvl_offset_x, auv_dvl_offset_y, auv_dvl_offset_z]))
    position_dvl_dvlref = position_auv_dvlref + dvl_offset_NWU

    #DVL QUATERNION
    q_NWU_auv = np.quaternion(pose_NWU_auv.orientation.w, pose_NWU_auv.orientation.x, pose_NWU_auv.orientation.y, pose_NWU_auv.orientation.z)
    q_dvlref_dvl = q_NWU_dvlref.inverse() * q_NWU_auv * q_dvl_auv.inverse()
    # euler_dvlref_auv = quaternion.as_euler_angles(q_dvlref_dvl)
    euler_dvlref_dvl = transformations.euler_from_quaternion([q_dvlref_dvl.x, q_dvlref_dvl.y, q_dvlref_dvl.z, q_dvlref_dvl.w])


    dvl_msg = DeadReckonReport()

    dvl_msg.x = position_dvl_dvlref[0]
    dvl_msg.y = position_dvl_dvlref[1]
    dvl_msg.z = position_dvl_dvlref[2]
    dvl_msg.std = 0.0
    dvl_msg.status = 1
    dvl_msg.roll = euler_dvlref_dvl[0] * DEG_PER_RAD
    dvl_msg.pitch = euler_dvlref_dvl[1] * DEG_PER_RAD
    dvl_msg.yaw = euler_dvlref_dvl[2] * DEG_PER_RAD
    pub_dvl_sensor.publish(dvl_msg)
    
    # # DEPTH FRAME: NWU
    # # DEPTH SENSOR
    depth_msg = pose_NWU_auv.position.z
    pub_depth_sensor.publish(depth_msg)

def cb_sim_imu(data):
    # IMU FRAME:
    #   x: down
    #   y: east
    #   z: south
    sbg_quat_msg = SbgEkfQuat()

    q_gazeboImuRef_gazeboImu = np.quaternion(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
    q_gazeboImuRef_imu = q_gazeboImuRef_gazeboImu * q_gazeboImu_imu
    q_NED_imu = q_NED_NWU * q_NWU_gazeboImuRef * q_gazeboImuRef_imu

    sbg_quat_msg.quaternion = Quaternion(q_NED_imu.x, q_NED_imu.y, q_NED_imu.z, q_NED_imu.w)


    sbg_data_msg = SbgImuData()
    
    ang_vel = quaternion.rotate_vectors(q_gazeboImu_imu.inverse(), np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]))
    sbg_data_msg.gyro = Vector3(ang_vel[0], ang_vel[1], ang_vel[2])

    pub_imu_quat_sensor.publish(sbg_quat_msg)
    pub_imu_data_sensor.publish(sbg_data_msg)

if __name__ == '__main__':

    rospy.init_node('bridge')
    
    DEG_PER_RAD = 180 / np.pi
    Q_IMUNOMINAL_AUV = np.quaternion(0, 1, 0, 0)

    q_dvlnominal_dvl_w = rospy.get_param("q_dvlnominal_dvl_w")
    q_dvlnominal_dvl_x = rospy.get_param("q_dvlnominal_dvl_x")
    q_dvlnominal_dvl_y = rospy.get_param("q_dvlnominal_dvl_y")
    q_dvlnominal_dvl_z = rospy.get_param("q_dvlnominal_dvl_z")
    q_dvlnominal_dvl = np.quaternion(q_dvlnominal_dvl_w,q_dvlnominal_dvl_x,q_dvlnominal_dvl_y,q_dvlnominal_dvl_z)  
    q_dvlnominal_auv = np.quaternion(0,1,0,0)

    auv_dvl_offset_x = rospy.get_param("auv_dvl_offset_x")
    auv_dvl_offset_y = rospy.get_param("auv_dvl_offset_y")
    auv_dvl_offset_z = rospy.get_param("auv_dvl_offset_z")
    
    q_imunominal_imu_w = rospy.get_param("q_imunominal_imu_w")
    q_imunominal_imu_x = rospy.get_param("q_imunominal_imu_x")
    q_imunominal_imu_y = rospy.get_param("q_imunominal_imu_y")
    q_imunominal_imu_z = rospy.get_param("q_imunominal_imu_z")
    
    # REFERENCE FRAME DEFINITIONS
    
    q_NED_NWU = np.quaternion(0, 1, 0, 0)
    q_NWU_dvlref = np.quaternion(0,1,0,0)

    q_imunominal_imu = np.quaternion(q_imunominal_imu_w, q_imunominal_imu_x, q_imunominal_imu_y, q_imunominal_imu_z)
    q_imu_auv = q_imunominal_imu.inverse() * Q_IMUNOMINAL_AUV

    q_auv_gazeboImu = np.quaternion(0, -0.70710678, 0, 0.70710678)
    q_NWU_gazeboImuRef = np.quaternion(0, -0.70710678, 0, 0.70710678)
    q_gazeboImu_imu = q_auv_gazeboImu.inverse() * q_imu_auv.inverse()

    q_dvl_auv = q_dvlnominal_dvl.conjugate() * q_dvlnominal_auv

    # simulate propulsion thrusters
    pub_t1 = rospy.Publisher('/model/clarke/joint/thruster1_joint/cmd_pos', Float64, queue_size=1)
    pub_t2 = rospy.Publisher('/model/clarke/joint/thruster2_joint/cmd_pos', Float64, queue_size=1) 
    pub_t3 = rospy.Publisher('/model/clarke/joint/thruster3_joint/cmd_pos', Float64, queue_size=1)
    pub_t4 = rospy.Publisher('/model/clarke/joint/thruster4_joint/cmd_pos', Float64, queue_size=1)
    pub_t5 = rospy.Publisher('/model/clarke/joint/thruster5_joint/cmd_pos', Float64, queue_size=1)
    pub_t6 = rospy.Publisher('/model/clarke/joint/thruster6_joint/cmd_pos', Float64, queue_size=1)
    pub_t7 = rospy.Publisher('/model/clarke/joint/thruster7_joint/cmd_pos', Float64, queue_size=1)
    pub_t8 = rospy.Publisher('/model/clarke/joint/thruster8_joint/cmd_pos', Float64, queue_size=1)
    rospy.Subscriber('/propulsion/forces', ThrusterForces, cb_thrusters)
    
    pub_dvl_sensor = rospy.Publisher('/sensors/dvl/pose', DeadReckonReport, queue_size=1)

    pub_depth_sensor = rospy.Publisher('/sensors/depth/z', Float64, queue_size=1)

    pub_imu_quat_sensor = rospy.Publisher('/sensors/imu/quaternion', SbgEkfQuat, queue_size=1)
    pub_imu_data_sensor = rospy.Publisher('/sensors/imu/angular_velocity', SbgImuData, queue_size=1)

    rospy.Subscriber('/imu', Imu, cb_sim_imu)
    rospy.Subscriber('/world/quali/dynamic_pose/info', PoseArray, cb_sim_dvl_depth)

    rospy.spin()
