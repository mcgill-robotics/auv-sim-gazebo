#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import numpy as np
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge

def rbg_callback(msg):
    global rgb
    temp = bridge.imgmsg_to_cv2(msg)
    rgb = temp/255

def camera_info_callback(msg):
    global fx, fy, cx, cy, width, height, x_over_z_map, y_over_z_map, convert_map
    fx = msg.K[0]
    fy = msg.K[4]
    cy = msg.K[2]
    cx = msg.K[5]
    width = msg.width
    height = msg.height

    # print(cx, cy, width/2, height/2)

    # cx = int(width/2)
    # cy = int(height/2)
    # print(msg.K[2], msg.K[5])

    u_map = np.tile(np.arange(width),(height,1)) +1
    v_map = np.tile(np.arange(height),(width,1)).T +1

    x_over_z_map = (cx - u_map) / fx
    y_over_z_map = (cy - v_map) / fy

    convert_map = np.sqrt(1 + x_over_z_map**2 + y_over_z_map**2)

def convert_from_uvd(msg):
    if convert_map is not None and rgb is not None:
        d = bridge.imgmsg_to_cv2(msg)
        # replace nan with inf
        time = rospy.Time.now()
        z_map = np.nan_to_num(d, nan=np.inf)
        # print amount of np.nan in x_over_z_map and y_over_z_map
        x_map = x_over_z_map * z_map
        y_map = y_over_z_map * z_map
        combined = np.stack((z_map, x_map, y_map), axis=2)
    
        xyz_rbg_img = np.zeros((height, width, 6))
        xyz_rbg_img[:, :, 0:3] = combined
        xyz_rbg_img[:, :, 3:6] = rgb
        xyz_rbg_img = xyz_rbg_img.reshape((width*height, 6))
        xyz_rbg_img = xyz_rbg_img.astype(np.float32)
        fields = [point_cloud2.PointField('x', 0, point_cloud2.PointField.FLOAT32, 1),
                    point_cloud2.PointField('y', 4, point_cloud2.PointField.FLOAT32, 1),
                    point_cloud2.PointField('z', 8, point_cloud2.PointField.FLOAT32, 1),
                    point_cloud2.PointField('r', 12, point_cloud2.PointField.FLOAT32, 1),
                    point_cloud2.PointField('g', 16, point_cloud2.PointField.FLOAT32, 1),
                    point_cloud2.PointField('b', 20, point_cloud2.PointField.FLOAT32, 1)]
        pub_msg = point_cloud2.create_cloud(msg.header, fields, xyz_rbg_img)
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

    rgb = None


    depth_sub = rospy.Subscriber('vision/front_cam/depth', Image, convert_from_uvd, queue_size=1)
    rgb_sub = rospy.Subscriber('vision/front_cam/image_rgb', Image, rbg_callback, queue_size=1)
    point_cloud_pub = rospy.Publisher('vision/front_cam/point_cloud', PointCloud2, queue_size=1)

    rospy.spin()