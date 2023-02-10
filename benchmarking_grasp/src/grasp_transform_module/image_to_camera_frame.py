'''
Author: Bharath Kumar 
Email: kumar7bharath@gmail.com
'''

import rospy
import cv_bridge
import numpy as np
import cv2
from tf import transformations as tft

import geometry_msgs.msg as gmsg
from sensor_msgs.msg import Image, CameraInfo
from benchmarking_msgs.srv import GraspPrediction, GraspPredictionResponse, Grasp2DPrediction, \
                    Grasp2DPredictionRequest  

class ImageToCameraFrame:
    '''
    Transforms the detected grasp in 2D depth image frame to 3D coordinates in the camera frame
    '''
    def __init__(self, sim_mode=True, crop=True):
        self.sim_mode = sim_mode
        self.crop = crop

        self.bridge = cv_bridge.CvBridge()

        self.curr_depth_img = None
        self.curr_rgb_img = None
        self.depth_scale = 1

        # Variables makes sure of the following 
        # 1. Transform service on call waits until the depth and rgb images are received
        # 2. The depth and the rgb image are only updated when the service is called
        self.waiting = False
        self.received = False
        self.rgb_received = False

        # TODO parametrize, check the actual value
        self.cam_fov = 65.5

        # Indies 0, 2: v and 1, 3: u
        # self.crop_size = [110, 197, 720, 1083] 
        # self.crop_size = [110, 295, 720, 1181] 
        self.crop_size = [140, 325, 690, 1151] 

        # Get camera info and subscribe to rgb and depth images
        if self.sim_mode:
            cam_info_topic = '/panda_camera/rgb/camera_info'
            rospy.Subscriber('/panda_camera/depth/image_raw', Image, self._depth_img_callback, queue_size=1)
            rospy.Subscriber('/panda_camera/rgb/image_raw', Image, self._rgb_img_callback, queue_size=1)
        else:
            self.depth_scale = 0.001  # Depth scale of realsense
            cam_info_topic = '/camera/aligned_depth_to_color/camera_info'
            # rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self._depth_img_callback, queue_size=1)
            rospy.Subscriber('/camera/aligned_depth_to_color/depth_completed', Image, self._depth_img_callback, queue_size=1)
            rospy.Subscriber('/camera/color/image_raw', Image, self._rgb_img_callback, queue_size=1)

        # To manually enter the camera matrix
        # K = [886.8075059058992, 0.0, 512.5, 0.0, 886.8075059058992, 512.5, 0.0, 0.0, 1.0]
        # self.cam_K = np.array(K).reshape((3, 3))

        # Get camera matrix from info topic        
        camera_info_msg = rospy.wait_for_message(cam_info_topic, CameraInfo)
        self.cam_K = np.array(camera_info_msg.K).reshape((3, 3))
        # rospy.loginfo("[Grasp Transform] Camera matrix extraction successful")

        # Service that transforms the coordinates
        rospy.Service('coords_in_cam', GraspPrediction, self.transform_coords_cb)

        # Topic for grasp visualization (Useful for debugging)
        self.img_pub = rospy.Publisher('~visualisation', Image, queue_size=1)

        # Publishes cropped results (Useful for debugging)
        self.rgb_cropped_pub = rospy.Publisher("cropped_rgb", Image, queue_size=10)
        self.depth_cropped_pub = rospy.Publisher("cropped_depth", Image, queue_size=10) 
        rospy.loginfo("[Image to Camera] Node loaded successfully")

    def _depth_img_callback(self, msg):
        '''
        Subscribes depth image from the corresponding topic 
        '''
        img = self.bridge.imgmsg_to_cv2(msg)

        if not self.waiting:
          return
        
        if self.crop:
            self.curr_depth_img = img[self.crop_size[0]:self.crop_size[2], self.crop_size[1]:self.crop_size[3]]
            self.depth_cropped_pub.publish(self.bridge.cv2_to_imgmsg(self.curr_depth_img))
        else:
            self.curr_depth_img = img 

        self.received = True

    def _rgb_img_callback(self, msg):
        '''
        Subscribes rgb image from the corresponding topic
        '''
        img = self.bridge.imgmsg_to_cv2(msg)

        if not self.waiting:
            return

        if self.crop:
            self.curr_rgb_img = img[self.crop_size[0]:self.crop_size[2], self.crop_size[1]:self.crop_size[3]]
            self.rgb_cropped_pub.publish(self.bridge.cv2_to_imgmsg(self.curr_rgb_img, encoding='rgb8'))
        else:
            self.curr_rgb_img = img

        self.rgb_received = True

    def transform_coords_cb(self, req):
        '''
        Service callback that transforms the coordinates and returns the resulting pose
        '''
        # Waits until a new image is received
        self.waiting = True
      
        rospy.loginfo("[Grasp Transform] waiting for the next image")
        while not self.received or not self.rgb_received:
            rospy.sleep(0.01)
        rospy.loginfo("[Grasp Transform] next image received")
    
        # RGB/Depth image is no longer updated
        self.waiting = False
        self.received = False
        self.rgb_received = False

        depth = self.curr_depth_img.copy()
        rgb = self.curr_rgb_img.copy()

        # Perform grasp prediction, Get coords in image frame
        center, width, quality, angle = self.predict_grasp(depth, rgb)

        precrop_center = center.copy() 

        # Accounting for crop 
        center[0] = self.crop_size[0] + center[0]
        center[1] = self.crop_size[1] + center[1]

        # Warping the angle
        angle = (angle + np.pi/2) % np.pi - np.pi/2  # Wrap [-np.pi/2, np.pi/2]

        # check for nearby depths and assign the max of the depths
        # _, z = self.find_depth_from_rect(depth, int(precrop_center[1]), int(precrop_center[0]), angle)

        # If you dont want to use the above functionality
        z = depth[int(precrop_center[0])][int(precrop_center[1])]*self.depth_scale

        # TODO u = y, v = x, where x,y are matrix coords and u,v are image coords
        coords_in_cam = np.linalg.inv(self.cam_K)@np.array([[center[1]], [center[0]], [1]])
        coords_in_cam = coords_in_cam*z/coords_in_cam[2][0]

        # Transform grasp angle
        angle_vec_in_cam = np.linalg.inv(self.cam_K)@np.array([[np.cos(angle)], [np.sin(angle)], [1]])
        angle_in_cam = np.arctan2(angle_vec_in_cam[1], angle_vec_in_cam[0])

        # Response message
        ret = GraspPredictionResponse()
        ret.success = True
        g = ret.best_grasp
        g.pose.position.x = coords_in_cam[0][0]
        g.pose.position.y = coords_in_cam[1][0]
        g.pose.position.z = coords_in_cam[2][0]
        
        g.pose.orientation = self.list_to_quaternion(tft.quaternion_from_euler(0, 0, angle_in_cam))
        g.width = width
        g.quality = quality

        self.draw_angled_rect(rgb, precrop_center[1], precrop_center[0], angle) 

        print(f"Grasp in camera frame x:{g.pose.position.x}, y:{g.pose.position.y}, z:{g.pose.position.z}")

        return ret

    def predict_grasp(self, depth, rgb):
        '''
        Calls the service responsible for predicting grasp in the image frame
        '''
        rospy.wait_for_service("grasp_service/predict", timeout=30)

        try:
            srv_handle = rospy.ServiceProxy("grasp_service/predict", Grasp2DPrediction)
            
            request_msg = Grasp2DPredictionRequest()
            request_msg.depth_image = self.bridge.cv2_to_imgmsg(depth)
            request_msg.rgb_image = self.bridge.cv2_to_imgmsg(rgb)
            
            response = srv_handle(request_msg)
            center = [response.best_grasp.px, response.best_grasp.py]
            width = response.best_grasp.width
            quality = response.best_grasp.quality
            angle = response.best_grasp.angle

            return center, width, quality, angle

        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)

    def draw_angled_rect(self, image, x, y, angle, width = 200, height = 100):
        """
        Draws bounding box for visualization
        """

        b = np.cos(angle) * 0.5
        a = np.sin(angle) * 0.5
        
        # For coloured images
        display_image = image.copy()

        pt0 = (int(x - a * height - b * width), int(y + b * height - a * width))
        pt1 = (int(x + a * height - b * width), int(y - b * height - a * width))
        pt2 = (int(2 * x - pt0[0]), int(2 * y - pt0[1]))
        pt3 = (int(2 * x - pt1[0]), int(2 * y - pt1[1]))

        cv2.line(display_image, pt0, pt1, (255, 0, 0), 5)
        cv2.line(display_image, pt1, pt2, (0, 0, 0), 5)
        cv2.line(display_image, pt2, pt3, (255, 0, 0), 5)
        cv2.line(display_image, pt3, pt0, (0, 0, 0), 5)
        cv2.circle(display_image, ((pt0[0] + pt2[0])//2, (pt0[1] + pt2[1])//2), 3, (0, 0, 0), -1)

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(display_image, encoding="rgb8"))

    def find_depth_from_rect(self, depth_image, x, y, angle, width=180, height = 100):
        """
        Finds the top most point inside the bounding box
        """
        # Orientation of the bounding box
        b = np.cos(angle) * 0.5
        a = np.sin(angle) * 0.5

        # Corners of the bounding box
        pt0 = (int(x - a * height - b * width), int(y + b * height - a * width))
        pt1 = (int(x + a * height - b * width), int(y - b * height - a * width))
        pt2 = (int(2 * x - pt0[0]), int(2 * y - pt0[1]))
        pt3 = (int(2 * x - pt1[0]), int(2 * y - pt1[1]))

        mask = np.zeros((depth_image.shape), dtype=np.uint8)
        
        pts = np.array( [[[pt0[0], pt0[1]], 
                          [pt1[0], pt1[1]],
                          [pt2[0], pt2[1]],
                          [pt3[0], pt3[1]]]], dtype=np.int32)
        cv2.fillPoly(mask, pts, 1)
        values = depth_image[np.where((mask == 1))]

        return max(values), min(values)

    def list_to_quaternion(self, l):
        q = gmsg.Quaternion()
        q.x = l[0]
        q.y = l[1]
        q.z = l[2]
        q.w = l[3]
        return q
