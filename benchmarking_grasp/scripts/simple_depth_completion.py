#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial import KDTree as KDTree
import cv2
import cv_bridge

from sensor_msgs.msg import Image

class DepthCompletion:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self._depth_img_cb, queue_size=1)
        self.depth_complete_pub = rospy.Publisher("/camera/aligned_depth_to_color/depth_completed", Image, queue_size=10)
        self.depth_complete_norm_pub = rospy.Publisher("/camera/aligned_depth_to_color/depth_completed_norm", Image, queue_size=10)

    def complete_depth(self, image):
        '''
        replaces nonzero pixels with the nearest nonzero pixel
        '''
        
        valid_pixels = np.argwhere(image != 0)
        invalid_pixels = np.argwhere(image == 0)
        
        kdtree = KDTree(valid_pixels)
        _, pre_indices = kdtree.query(invalid_pixels, k=1)
        
        indices = valid_pixels[pre_indices]
        image[invalid_pixels[:, 0], invalid_pixels[:, 1]] = image[indices[:, 0], indices[:, 1]]
        
        return image

    def _depth_img_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        img = self.complete_depth(img.copy())
        norm_image = cv2.normalize(img.copy(), None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        self.depth_complete_pub.publish(self.bridge.cv2_to_imgmsg(img))
        self.depth_complete_norm_pub.publish(self.bridge.cv2_to_imgmsg(norm_image))

if __name__ == "__main__":
    rospy.init_node("simple_depth_completion", log_level=rospy.INFO)
    depth_completion = DepthCompletion()

    rospy.spin()
    