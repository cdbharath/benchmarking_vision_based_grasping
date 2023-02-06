#!/usr/bin/env python3

import rospy
from grasp_transform_module.image_to_camera_frame import ImageToCameraFrame

if __name__ == "__main__":
    rospy.init_node('image_to_camera_node')
    cam_to_world_module = ImageToCameraFrame(sim_mode=False, crop=False)
    rospy.spin()
