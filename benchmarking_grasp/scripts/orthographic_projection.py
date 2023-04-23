#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt

class DepthImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/panda_camera/depth/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/panda_camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        self.pers_pub = rospy.Publisher('/perspective_image', Image, queue_size=1)
        self.ortho_pub = rospy.Publisher('/orthographic_image', Image, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.process_depth_image)
        
        self.depth_image = None
        self.camera_matrix = None

    def depth_callback(self, depth_data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

    def camera_info_callback(self, camera_info):
        self.camera_matrix = np.reshape(camera_info.K, (3, 3))
        print(self.camera_matrix)

    def process_depth_image(self, data):
        if self.depth_image is not None and self.camera_matrix is not None:
            h, w = self.depth_image.shape[:2]
            
            xy = np.meshgrid(np.arange(w), np.arange(h))
            x = xy[0].reshape(-1)
            y = xy[1].reshape(-1)
            z = self.depth_image[x, y]
            
            xyz_cam = np.linalg.inv(self.camera_matrix).dot(np.vstack((y, x, np.ones_like(x))))
            x_cam, y_cam, z_cam = xyz_cam 
                       
            # plt.scatter(y_cam, z)
            # plt.axis('square')    
            # plt.savefig('test.png')
            
            ortho_image = np.ones((h, w))*1000
            x_ortho = ((y_cam - np.min(y_cam)) * (w - 1) / (np.max(y_cam) - np.min(y_cam))).astype(int)
            y_ortho = ((x_cam - np.min(x_cam)) * (h - 1) / (np.max(x_cam) - np.min(x_cam))).astype(int)
            
            for x_ortho_, y_ortho_, z in zip(x_ortho, y_ortho, z):
                if z > 0 and z < ortho_image[x_ortho_, y_ortho_]:
                    ortho_image[x_ortho_, y_ortho_] = z
            
            ortho_image[ortho_image == 1000] = 0
            ortho_image[ortho_image == 0] = np.max(ortho_image)
            
            img_norm = cv2.normalize(ortho_image, None, 0, 255, cv2.NORM_MINMAX)
            img_8uc1 = img_norm.astype(np.uint8)
            msg_8uc1 = self.bridge.cv2_to_imgmsg(img_8uc1, encoding='mono8')

            # ortho_image[x_ortho, y_ortho] = z 
            
            print(np.min(x_ortho), np.min(y_ortho), np.max(x_ortho), np.max(y_ortho), w, h)
            self.pers_pub.publish(self.bridge.cv2_to_imgmsg(self.depth_image, encoding='passthrough'))
            self.ortho_pub.publish(msg_8uc1)
            
if __name__ == '__main__':
    rospy.init_node('depth_image_processor')
    dip = DepthImageProcessor()
    rospy.spin()
