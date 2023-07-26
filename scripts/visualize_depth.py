#!/usr/bin/env python3

import struct

import cv2
import message_filters
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import torch
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import StereoCameraModel
from kornia.geometry import depth_to_3d
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from stereo_msgs.msg import DisparityImage



def camera_info_callback(msg: CameraInfo):
    global ros_cam_info
    ros_cam_info = msg


def registered_callback(rgb_msg: Image, disp_msg: DisparityImage):
    # Project the disparity image to 3D
    baseline = 64e-3  # 64 mm
    global ros_cam_info
    fx = ros_cam_info.P[0]
    cx = ros_cam_info.P[2]
    cy = ros_cam_info.P[6]
    T = baseline
    Q = np.array([[1, 0, 0, -cx], [0, 1, 0, -cy], [0, 0, 0, fx], [0, 0, -1 / T, 0]])
    bridge = CvBridge()
    # Convert the images to OpenCV format
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth_image = bridge.imgmsg_to_cv2(disp_msg.image, desired_encoding="passthrough")

    # Convert the depth image to a 3D point cloud
    points = cv2.reprojectImageTo3D(
        depth_image, Q
    )  # Assuming Q is your reprojection matrix

    # Flatten and convert the points
    points = points.reshape(-1, 3)

    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("rgb", 12, PointField.UINT32, 1),
    ]

    # Flatten the RGB image and pack the channels into a single uint32
    rgb_image = rgb_image.reshape(-1, 3)
    rgb_packed = (
        np.left_shift(rgb_image[:, 2].astype(np.uint32), 16)
        | np.left_shift(rgb_image[:, 1].astype(np.uint32), 8)
        | rgb_image[:, 0].astype(np.uint32)
    )

    # Stack the 3D points with the packed RGB values
    point_cloud_arr = np.zeros(
        len(points),
        dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("rgb", np.uint32),
        ],
    )
    point_cloud_arr["x"] = points[:, 0]
    point_cloud_arr["y"] = points[:, 1]
    point_cloud_arr["z"] = points[:, 2]
    point_cloud_arr["rgb"] = rgb_packed

    # Create the header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "zedm_left_camera_frame"

    # Create the point cloud message
    point_cloud_msg = pc2.create_cloud(header, fields, point_cloud_arr)

    # Publish the colored point cloud message...
    pointcloud_pub = rospy.Publisher(
        "/zedm/zed_node/pointcloud/registered_pointcloud", PointCloud2, queue_size=10
    )
    pointcloud_pub.publish(point_cloud_msg)


def main():
    global ros_cam_info
    rospy.init_node("depth_projection", anonymous=True)
    disp_sub = message_filters.Subscriber(
        "/zedm/zed_node/disparity/disparity_image", DisparityImage
    )
    rgb_sub = message_filters.Subscriber(
        "/zedm/zed_node/left_raw/image_raw_color", Image
    )
    ros_cam_info_sub = rospy.Subscriber(
        "/zedm/zed_node/left/camera_info", CameraInfo, camera_info_callback
    )

    ts = message_filters.ApproximateTimeSynchronizer(
        [rgb_sub, disp_sub], queue_size=10, slop=0.1
    )
    ts.registerCallback(registered_callback)

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    main()
