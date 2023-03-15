#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import cv_bridge
import yaml
import rospkg
import os

from sensor_msgs.msg import Image

class TestROI:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.normalize = False

        self.rospack = rospkg.RosPack()
        self.yaml_package_name = "benchmarking_grasp"
        self.yaml_package_path = os.path.join(self.rospack.get_path(self.yaml_package_name), "config/configuration.yaml")
        params = yaml.load(open(self.yaml_package_path, "r"), Loader=yaml.FullLoader)

        self.crop_size = params["crop_size"]

        depth_image_topic = params["depth_image"]
        rgb_image_topic = params["rgb_image"]

        depth_crop_topic = '/depth_crop'
        rgb_crop_topic = '/rgb_crop'

        rospy.Subscriber(depth_image_topic, Image, self._depth_img_cb, queue_size=1)
        rospy.Subscriber(rgb_image_topic, Image, self._rgb_img_cb, queue_size=1)

        self.depth_crop_pub = rospy.Publisher(depth_crop_topic, Image, queue_size=1)
        self.rgb_crop_pub = rospy.Publisher(rgb_crop_topic, Image, queue_size=1)

    def _depth_img_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        img_norm = img[self.crop_size[0]:self.crop_size[2], self.crop_size[1]:self.crop_size[3]]
        img_norm = self.normalize_depth(img_norm)

        self.depth_crop_pub.publish(self.bridge.cv2_to_imgmsg(img_norm))

    def _rgb_img_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        roi_img = cv2.rectangle(img, (self.crop_size[1], self.crop_size[0]), (self.crop_size[3], self.crop_size[2]), (0, 255, 0), 2) 

        self.rgb_crop_pub.publish(self.bridge.cv2_to_imgmsg(roi_img, encoding="rgb8"))

    def normalize_depth(self, depth_image):
        normalized_depth_image = ((depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))) * 255
        normalized_depth_image = np.uint8(normalized_depth_image)
        return normalized_depth_image

if __name__ == "__main__":
    rospy.init_node("test_roi", log_level=rospy.INFO)
    test_roi = TestROI()

    rospy.spin()
    