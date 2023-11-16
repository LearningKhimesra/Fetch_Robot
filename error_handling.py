#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge
import cv2
import numpy as np

class PoseEstimator:

    def __init__(self):
        self.bridge = CvBridge()

        # Camera intrinsics
        self.camera_intrinsics = None

        # Subscribers
        self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/head_camera/depth_registered/image_raw", Image, self.depth_callback)
        self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bbox_callback)
        self.camera_info_sub = rospy.Subscriber("/head_camera/depth_registered/camera_info", CameraInfo, self.camera_info_callback)

        self.rgb_image = None
        self.depth_image = None
        self.bboxes = []

    def rgb_callback(self, data):
        self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")

    def camera_info_callback(self, data):
        self.camera_intrinsics = np.array([[data.K[0], 0, data.K[2]],
                                           [0, data.K[4], data.K[5]],
                                           [0, 0, 1]])

    def bbox_callback(self, bboxes):
        if not self.validate_data():
            return

        self.bboxes = bboxes.bounding_boxes

        for bbox in self.bboxes:
            xmin, ymin, xmax, ymax = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax
            depth_roi = self.depth_image[ymin:ymax, xmin:xmax]
            median_depth = np.nanmedian(depth_roi[depth_roi > 0])

            if np.isnan(median_depth):
                rospy.logwarn("Invalid depth value. Skipping object: {}".format(bbox.Class))
                continue

            cx = (xmin + xmax) / 2.0
            cy = (ymin + ymax) / 2.0

            X = (cx - self.camera_intrinsics[0, 2]) * median_depth / self.camera_intrinsics[0, 0]
            Y = (cy - self.camera_intrinsics[1, 2]) * median_depth / self.camera_intrinsics[1, 1]
            Z = median_depth

            rospy.loginfo("Object: {} | Depth: {} | Bounding Box: [{}, {}, {}, {}] | 3D Pose: [{}, {}, {}]".format(
                bbox.Class, median_depth, xmin, ymin, xmax, ymax, X, Y, Z
            ))

    def validate_data(self):
        if self.rgb_image is None or self.depth_image is None or self.camera_intrinsics is None:
            rospy.logwarn("Missing data for pose estimation. Please ensure all sensors are functioning.")
            return False
        return True

if __name__ == "__main__":
    rospy.init_node('fetch_pose_estimator', anonymous=True)
    pe = PoseEstimator()
    rospy.spin()

