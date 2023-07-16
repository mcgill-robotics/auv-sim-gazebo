#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge

def convert_from_uvd(msg):
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
    width = 512
    height = 341
    fx = 277
    fy = 277
    cx = 160
    cy = 120

    u_map = np.tile(np.arange(width),(height,1)) +1
    v_map = np.tile(np.arange(height),(width,1)).T +1

    x_over_z_map = (cx - u_map) / fx
    y_over_z_map = (cy - v_map) / fy

    convert_map = np.sqrt(1 + x_over_z_map**2 + y_over_z_map**2)

    depth_sub = rospy.Subscriber('vision/front_cam/depth', Image, convert_from_uvd, queue_size=1)
    point_cloud_pub = rospy.Publisher('vision/front_cam/point_cloud', PointCloud2, queue_size=1)

    rospy.spin()