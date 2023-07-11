#!/usr/bin/env python3

import rospy
from sensor_msgs import PointCloud2, Image
import numpy as np
from sensor_msgs import point_cloud2
from pcl_msgs.msg import PCLPointField

def convert_from_uvd(d):
    
 
    z_map = d / convert_map
    x_map = x_over_z_map * z_map
    y_map = y_over_z_map * z_map

    combined = np.stack((x_map, y_map, z_map), axis=2)
    flat = combined.reshape(-1, 3)
    msg = array_to_pointcloud2(flat)
    point_cloud_pub.publish(msg)

def array_to_pointcloud2(points):
    fields = [
        PCLPointField(
            name='x',
            offset=0,
            datatype=PCLPointField.FLOAT32,
            count=1
        ),
        PCLPointField(
            name='y',
            offset=4,
            datatype=PCLPointField.FLOAT32,
            count=1
        ),
        PCLPointField(
            name='z',
            offset=8,
            datatype=PCLPointField.FLOAT32,
            count=1
        )
    ]

    # Create the point cloud message
    msg = point_cloud2.create_cloud_xyz32(msg.header, fields, points)

    return msg

if __name__ == "__main__":
    rospy.init_node('point_cloud_sim')

    width = 512
    height = 341
    fx = 277
    fy = 277
    cx = 160
    cy = 120

    u_map = np.tile(np.arange(width),(height,1))
    v_map = np.tile(np.arange(height),(width,1)).T

    x_over_z_map = (cx - u_map) / fx
    y_over_z_map = (cy - v_map) / fy

    convert_map = np.sqrt(1 + x_over_z_map**2 + y_over_z_map**2)

    depth_sub = rospy.Subscriber('vision/front_cam/depth', Image, convert_from_uvd)
    point_cloud_pub = rospy.Publisher('vision/front_cam/point_cloud', PointCloud2, queue_size=1)

    rospy.spin()