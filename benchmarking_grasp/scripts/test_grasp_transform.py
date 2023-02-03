#!/usr/bin/env python3

import rospy
from grasp_transform_module.grasp_transform import GraspTransform

if __name__ == "__main__":
    rospy.init_node('grasp_transform')
    grasp_Service = GraspTransform(sim_mode=False, crop=True)
    rospy.spin()
