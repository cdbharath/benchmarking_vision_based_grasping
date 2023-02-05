#!/usr/bin/env python3

import rospy
from grasp_transform_module.camera_to_world_frame import CameraToWorldFrame

if __name__ == "__main__":
    rospy.init_node('camera_to_world_node')
    cam_to_world_module = CameraToWorldFrame(sim_mode=False)
    rospy.spin()
