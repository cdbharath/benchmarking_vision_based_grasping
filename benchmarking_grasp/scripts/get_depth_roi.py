#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import cv_bridge
import yaml
import rospkg
import os

from sensor_msgs.msg import Image

def nothing(x):
    pass

class GetDepthROI:
    def __init__(self, sim_mode=False, depth_complete=True):
        self.bridge = cv_bridge.CvBridge()
        self.normalize = False
        self.depth_complete = True
        self.depth = None
        self.rgb = None

        self.rospack = rospkg.RosPack()
        self.yaml_package_name = "benchmarking_grasp"
        self.yaml_package_path = os.path.join(self.rospack.get_path(self.yaml_package_name), "config/configuration.yaml")
        self.params = yaml.safe_load(open(self.yaml_package_path, "r"))

        self.crop_size = self.params["crop_size"]

        if sim_mode:
            depth_image_topic = self.params["depth_image_sim"]
            rgb_image_topic = self.params["rgb_image_sim"]
        else:
            depth_image_topic = self.params["depth_image"]
            rgb_image_topic = self.params["rgb_image"]

            if self.depth_complete:
                depth_image_topic = self.params["depth_complete_image"]            

        rospy.Subscriber(depth_image_topic, Image, self._depth_img_cb, queue_size=1)
        rospy.Subscriber(rgb_image_topic, Image, self._rgb_img_cb, queue_size=1)

        cv2.namedWindow('Adjust ROI')
       
        # create trackbars for color change
        cv2.createTrackbar('X min','Adjust ROI',self.crop_size[1],2048,nothing)
        cv2.createTrackbar('X max','Adjust ROI',self.crop_size[3],2048,nothing)
        cv2.createTrackbar('Y min','Adjust ROI',self.crop_size[0],2048,nothing)
        cv2.createTrackbar('Y max','Adjust ROI',self.crop_size[2],2048,nothing)
   
        while True:
            # get current positions of four trackbars
            x_min = cv2.getTrackbarPos('X min','Adjust ROI')
            x_max = cv2.getTrackbarPos('X max','Adjust ROI')
            y_min = cv2.getTrackbarPos('Y min','Adjust ROI')
            y_max = cv2.getTrackbarPos('Y max','Adjust ROI')

            depth_norm = self.depth[y_min:y_max, x_min:x_max]
    
            max_val = np.max(depth_norm)
            min_val = np.min(depth_norm)
            # Set all depth values in this 1cm range to the same value
            # Assumes the view only has object and ground plane
            depth_norm[depth_norm > max((min_val + max_val)/2, max_val - 20)] = max_val
            depth_norm = self.normalize_depth(depth_norm)
    
            roi_img = cv2.rectangle(self.rgb, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)  
            roi_img = cv2.cvtColor(roi_img, cv2.COLOR_RGB2BGR)
            roi_img = cv2.resize(roi_img, (640, 480))
    
            cv2.imshow('ROI Depth', depth_norm)
            cv2.imshow('Adjust ROI', roi_img)

            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
       
        x_min = cv2.getTrackbarPos('X min','Adjust ROI')
        x_max = cv2.getTrackbarPos('X max','Adjust ROI')
        y_min = cv2.getTrackbarPos('Y min','Adjust ROI')
        y_max = cv2.getTrackbarPos('Y max','Adjust ROI')
        
        # self.set_config(x_min, x_max, y_min, y_max)
        print(f"crop_size: [{y_min}, {x_min}, {y_max}, {x_max}]")
        cv2.destroyAllWindows()  
   
    def _depth_img_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        self.depth = img.copy()

    def _rgb_img_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.rgb = img.copy()
        
    def normalize_depth(self, depth_image):
        normalized_depth_image = ((depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image))) * 255
        normalized_depth_image = np.uint8(normalized_depth_image)
        return normalized_depth_image

    def set_config(self, x_min, x_max, y_min, y_max):
        self.params['crop_size'] = [y_min, x_min, y_max, x_max]
        yaml.safe_dump(self.params, open(self.yaml_package_path, "w"), default_flow_style=False)

if __name__ == "__main__":
    rospy.init_node("get_depth_roi", log_level=rospy.INFO)
    test_roi = GetDepthROI(sim_mode=True, depth_complete=True)

    rospy.spin()
    