#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge

def camera_info_callback(msg):
    global fx, fy, cx, cy, width, height, convert_map
    fx = msg.K[0]
    fy = msg.K[4]
    cx = msg.K[2]
    cy = msg.K[5]
    width = msg.width
    height = msg.height

    u_map = np.tile(np.arange(width),(height,1)) +1
    v_map = np.tile(np.arange(height),(width,1)).T +1

    x_over_z_map = (cx - u_map) / fx
    y_over_z_map = (cy - v_map) / fy

    convert_map = np.sqrt(1 + x_over_z_map**2 + y_over_z_map**2)

def convert_from_uvd(msg):
    if convert_map is None: return
    time = rospy.Time(0)
    d = bridge.imgmsg_to_cv2(msg)
    # replace nan with inf
    d = np.nan_to_num(d, nan=np.inf)
    z_map = d / convert_map
    # print amount of np.nan in x_over_z_map and y_over_z_map
    x_map = x_over_z_map * z_map
    y_map = y_over_z_map * z_map

    combined = np.stack((z_map, x_map, y_map), axis=2)

    combined = combined.reshape((512*341, 3))
    pub_msg = point_cloud2.create_cloud_xyz32(msg.header, combined)
    pub_msg.header.frame_id = "auv_base"
    pub_msg.header.stamp = time
    point_cloud_pub.publish(pub_msg)


if __name__ == "__main__":
    rospy.init_node('point_cloud_sim')
    bridge = CvBridge()

    camera_info_sub = rospy.Subscriber('vision/front_cam/camera_info', CameraInfo, camera_info_callback, queue_size=1)
    fx = None
    fy = None
    cx = None
    cy = None
    width = None
    height = None

    x_over_z_map = None
    y_over_z_map = None
    convert_map = None


    depth_sub = rospy.Subscriber('vision/front_cam/depth', Image, convert_from_uvd, queue_size=1)
    point_cloud_pub = rospy.Publisher('vision/front_cam/point_cloud', PointCloud2, queue_size=1)

    rospy.spin()