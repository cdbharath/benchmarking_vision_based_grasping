import time
import math
import numpy as np
import cv2
import matplotlib
from matplotlib.path import Path

import rospy
from tf import transformations as tft
import tf2_ros
import tf2_geometry_msgs
from tf import TransformerROS
import cv_bridge
from benchmarking_msgs.srv import GraspPrediction, GraspPredictionResponse, Grasp2DPrediction, Grasp2DPredictionResponse, Grasp2DPredictionRequest
from sensor_msgs.msg import Image, CameraInfo
import geometry_msgs.msg as gmsg

bridge = cv_bridge.CvBridge()
tfBuffer = None
listener = None

class GraspTransform:
    def __init__(self, sim_mode=True):
        self.sim_mode = sim_mode
        self.crop = True
        # self.crop_size = [0, 300, 720, 1050]
        self.crop_size = [0, 0, 480, 640] # 480, 640

        # Get the camera parameters
        if self.sim_mode:
            cam_info_topic = '/panda_camera/rgb/camera_info'
            self.camera_frame = 'panda_camera_optical_link'
        else:
            cam_info_topic = '/camera/aligned_depth_to_color/camera_info'
            self.camera_frame = 'camera_depth_optical_frame'        
        rospy.loginfo("waiting for camera topic: %s", cam_info_topic)
        camera_info_msg = rospy.wait_for_message(cam_info_topic, CameraInfo)
        
        # To manually enter the camera matrix
        # K = [886.8075059058992, 0.0, 512.5, 0.0, 886.8075059058992, 512.5, 0.0, 0.0, 1.0]
        # self.cam_K = np.array(K).reshape((3, 3))
        
        self.cam_K = np.array(camera_info_msg.K).reshape((3, 3))
        rospy.loginfo("Camera matrix extraction successful")
        
        self.img_pub = rospy.Publisher('~visualisation', Image, queue_size=1)
        rospy.Service('~predict', GraspPrediction, self.compute_service_handler)
        
        self.base_frame = 'panda_link0'
        self.cam_fov = 65.5
        
        self.curr_depth_img = None
        self.curr_rgb_img = None
        self.curr_img_time = 0
        self.last_image_pose = None

        if self.sim_mode:
            rospy.Subscriber('/panda_camera/depth/image_raw', Image, self._depth_img_callback, queue_size=1)
            rospy.Subscriber('/panda_camera/rgb/image_raw', Image, self._rgb_img_callback, queue_size=1)
        else:
            rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self._depth_img_callback, queue_size=1)
            rospy.Subscriber('/camera/color/image_raw', Image, self._rgb_img_callback, queue_size=1)

        self.rgb_cropped_pub = rospy.Publisher("cropped_rgb", Image, queue_size=10)
        self.depth_cropped_pub = rospy.Publisher("cropped_depth", Image, queue_size=10) 

        self.waiting = False
        self.received = False
        self.rgb_received = False
    
    def _depth_img_callback(self, msg):
        # Doing a rospy.wait_for_message is super slow, compared to just subscribing and keeping the newest one.
        img = bridge.imgmsg_to_cv2(msg)

        if not self.waiting:
          return
        self.curr_img_time = time.time()
        self.last_image_pose = self.current_robot_pose(self.base_frame, self.camera_frame)
        
        if self.crop:
            self.curr_depth_img = img[self.crop_size[0]:self.crop_size[2], self.crop_size[1]:self.crop_size[3]]
            depth_crop = self.curr_depth_img.copy()
            depth_scale = np.max(np.abs(depth_crop))
            depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.
            normalized = (depth_crop*255).astype('uint8')
            self.depth_cropped_pub.publish(bridge.cv2_to_imgmsg(normalized))
        else:
            self.curr_depth_img = img 

        self.received = True

    def _rgb_img_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg)

        # if not self.waiting:
        #     return

        if self.crop:
            self.curr_rgb_img = img[self.crop_size[0]:self.crop_size[2], self.crop_size[1]:self.crop_size[3]]
            self.rgb_cropped_pub.publish(bridge.cv2_to_imgmsg(self.curr_rgb_img, encoding='rgb8'))
        else:
            self.curr_rgb_img = img

        self.rgb_received = True

    def compute_service_handler(self, req):
        self.waiting = True
        while not self.received or not self.rgb_received:
          rospy.sleep(0.01)
        self.waiting = False
        self.received = False
        self.rgb_received = False

        depth = self.curr_depth_img.copy()
        rgb = self.curr_rgb_img.copy()

        camera_pose = self.last_image_pose
        cam_p = camera_pose.position

        camera_rot = tft.quaternion_matrix(self.quaternion_to_list(camera_pose.orientation))[0:3, 0:3]

        rospy.loginfo("checking for grasps")

        # Do grasp prediction
        rospy.loginfo("waiting for service: grasp_service/predict")
        rospy.wait_for_service("grasp_service/predict")
        rospy.loginfo("Service call successful")

        srv_handle = rospy.ServiceProxy("grasp_service/predict", Grasp2DPrediction)
        
        request_msg = Grasp2DPredictionRequest()
        request_msg.depth_image = bridge.cv2_to_imgmsg(depth)
        request_msg.rgb_image = bridge.cv2_to_imgmsg(rgb)
        
        response = srv_handle(request_msg)
        center = response.best_grasp.px, response.best_grasp.py
        width = response.best_grasp.width
        quality = response.best_grasp.quality
        angle = response.best_grasp.angle
        rospy.loginfo("grasp found")

        # Convert from image frame to camera frame
        x = (center[1] - self.cam_K[0, 2])/self.cam_K[0, 0]
        y = (center[0] - self.cam_K[1, 2])/self.cam_K[1, 1]

        # check for nearby depths and assign the max of the depths
        # z = depth[int(center[0])][int(center[1])]
        z = self.find_depth(depth, center[0], center[1], angle, width, int(width*0.4))

        angle = (angle + np.pi/2) % np.pi - np.pi/2  # Wrap [-np.pi/2, np.pi/2]
                
        # Convert from camera frame to world frame 
        pos = np.dot(camera_rot, np.stack((x, y, z))).T + np.array([[cam_p.x, cam_p.y, cam_p.z]])

        ret = GraspPredictionResponse()
        ret.success = True
        g = ret.best_grasp
        g.pose.position.x = pos[0][0]
        g.pose.position.y = pos[0][1]
        g.pose.position.z = pos[0][2]
        
        g.pose.orientation = self.list_to_quaternion(tft.quaternion_from_euler(np.pi, 0, angle))
        g.width = width
        g.quality = quality

        # self.draw_angled_rect(rgb, center[0], center[1], angle, width, int(width*0.4))
        # self.draw_angled_rect(rgb, center[1], center[0], angle, width, int(width*0.4))
        self.draw_angled_rect(rgb, center[1], center[0], angle) # work around for width

        return ret

    def draw_angled_rect(self, image, x, y, angle, width = 200, height = 100):
        # print(x, y, angle, image.shape)
        _angle = -angle
        b = math.cos(_angle) * 0.5
        a = math.sin(_angle) * 0.5

        # gray_image = image.copy()
        # display_image = cv2.applyColorMap((gray_image * 255).astype(np.uint8), cv2.COLORMAP_BONE)
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

        self.img_pub.publish(bridge.cv2_to_imgmsg(display_image, encoding="rgb8"))

    def find_depth(self, depth_image, x, y, angle, width=180, height = 100):
        max_depth = float("inf")

        _angle = -angle
        b = math.cos(_angle) * 0.5
        a = math.sin(_angle) * 0.5

        pt0 = (int(x - a * height - b * width), int(y + b * height - a * width))
        pt1 = (int(x + a * height - b * width), int(y - b * height - a * width))
        pt2 = (int(2 * x - pt0[0]), int(2 * y - pt0[1]))
        pt3 = (int(2 * x - pt1[0]), int(2 * y - pt1[1]))

        p1 = np.array((pt0[0] + pt1[0], pt0[1] + pt1[1]))/2
        p2 = np.array((pt2[0] + pt3[0], pt2[1] + pt3[1]))/2

        p = p1
        d = p2-p1
        N = np.max(np.abs(d))
        s = d/N
        
        for ii in range(0,int(N)):
            p = p+s
            max_depth = min(max_depth, depth_image[int(p[0])][int(p[1])])
        
        p1 = np.array((pt1[0] + pt2[0], pt1[1] + pt2[1]))/2
        p2 = np.array((pt0[0] + pt3[0], pt0[1] + pt3[1]))/2
        
        p = p1
        d = p2-p1
        N = np.max(np.abs(d))
        s = d/N
        
        for ii in range(0,int(N)):
            p = p+s
            max_depth = min(max_depth, depth_image[int(p[0])][int(p[1])])
        
        return max_depth

    def list_to_quaternion(self, l):
        q = gmsg.Quaternion()
        q.x = l[0]
        q.y = l[1]
        q.z = l[2]
        q.w = l[3]
        return q

    def quaternion_to_list(self, quaternion):
        return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

    def current_robot_pose(self, reference_frame, base_frame):
        """
        Get the current pose of the robot in the given reference frame
            reference_frame         -> A string that defines the reference_frame that the robots current pose will be defined in
        """
        # Create Pose
        p = gmsg.Pose()
        p.orientation.w = 1.0

        # Transforms robots current pose to the base reference frame
        return self.convert_pose(p, base_frame, reference_frame)
      
    def _init_tf(self):
        # Create buffer and listener
        # Something has changed in tf that means this must happen after init_node
        global tfBuffer, listener
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
    
    def convert_pose(self, pose, from_frame, to_frame):
        """
        Convert a pose or transform between frames using tf.
            pose            -> A geometry_msgs.msg/Pose that defines the robots position and orientation in a reference_frame
            from_frame      -> A string that defines the original reference_frame of the robot
            to_frame        -> A string that defines the desired reference_frame of the robot to convert to
        """
        global tfBuffer, listener

        if tfBuffer is None or listener is None:
            self._init_tf()

        try:
            trans = tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
            return None

        spose = gmsg.PoseStamped()
        spose.pose = pose
        spose.header.stamp = rospy.Time().now
        spose.header.frame_id = from_frame

        p2 = tf2_geometry_msgs.do_transform_pose(spose, trans)

        return p2.pose


if __name__ == "__main__":
    rospy.init_node('grasp_transform')
    grasp_Service = GraspTransform()
    rospy.spin()
